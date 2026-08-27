#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "h264_camera_recorder.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr int kBufferCount = 4;
constexpr uint64_t kExposureRefreshIntervalNs = 200000000ULL;

struct TriggerEvent
{
  unsigned long long timestamp_ns;
  long long counter;
};

uint64_t timespec_to_ns(const timespec & ts)
{
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t timeval_to_ns(const timeval & tv)
{
  return static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL + static_cast<uint64_t>(tv.tv_usec) * 1000ULL;
}

uint64_t clock_ns(clockid_t clock_id)
{
  timespec ts {};
  if (clock_gettime(clock_id, &ts) != 0) {
    throw std::runtime_error(std::string("clock_gettime failed: ") + std::strerror(errno));
  }
  return timespec_to_ns(ts);
}

bool parse_local_iso_epoch_ns(const std::string & value, uint64_t * epoch_ns)
{
  std::tm tm {};
  std::istringstream input(value);
  input >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
  if (input.fail()) {
    return false;
  }
  tm.tm_isdst = -1;
  const time_t sec = std::mktime(&tm);
  if (sec < 0) {
    return false;
  }
  *epoch_ns = static_cast<uint64_t>(sec) * 1000000000ULL;
  return true;
}

void write_text_file(const std::string & path, const std::string & value)
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("open " + path + " failed: " + std::strerror(errno));
  }
  const ssize_t n = ::write(fd, value.data(), value.size());
  const int saved_errno = errno;
  ::close(fd);
  if (n != static_cast<ssize_t>(value.size())) {
    throw std::runtime_error("write " + path + " failed: " + std::strerror(saved_errno));
  }
}

class TimeMapper
{
public:
  explicit TimeMapper(std::string state_file)
  : state_file_(std::move(state_file))
  {
    epoch_wall_ns_ = clock_ns(CLOCK_REALTIME);
    epoch_mono_ns_ = clock_ns(CLOCK_MONOTONIC);
    fallback_epoch_wall_ns_ = epoch_wall_ns_;
    fallback_epoch_mono_ns_ = epoch_mono_ns_;
    reload();
  }

  void maybe_reload()
  {
    const uint64_t now_mono = clock_ns(CLOCK_MONOTONIC);
    if (now_mono - last_reload_mono_ns_ > 1000000000ULL) {
      reload();
    }
  }

  rclcpp::Time stamp_from_mono_ns(uint64_t mono_ns) const
  {
    const uint64_t wall_ns =
      epoch_wall_ns_ + static_cast<int64_t>(mono_ns) - static_cast<int64_t>(epoch_mono_ns_);
    return rclcpp::Time(static_cast<int64_t>(wall_ns), RCL_SYSTEM_TIME);
  }

  bool using_state_file() const { return using_state_file_; }

private:
  void reload()
  {
    last_reload_mono_ns_ = clock_ns(CLOCK_MONOTONIC);
    if (state_file_.empty()) {
      using_state_file_ = false;
      return;
    }

    std::ifstream input(state_file_);
    if (!input) {
      epoch_wall_ns_ = fallback_epoch_wall_ns_;
      epoch_mono_ns_ = fallback_epoch_mono_ns_;
      using_state_file_ = false;
      return;
    }

    std::unordered_map<std::string, std::string> values;
    std::string line;
    while (std::getline(input, line)) {
      const auto pos = line.find('=');
      if (pos == std::string::npos) {
        continue;
      }
      values[line.substr(0, pos)] = line.substr(pos + 1);
    }

    try {
      epoch_mono_ns_ = std::stoull(values.at("epoch_mono_ns"));
      const auto start_it = values.find("start_local");
      if (start_it != values.end()) {
        uint64_t start_epoch_ns = 0;
        if (!parse_local_iso_epoch_ns(start_it->second, &start_epoch_ns)) {
          throw std::runtime_error("invalid start_local");
        }
        epoch_wall_ns_ = start_epoch_ns;
      } else {
        epoch_wall_ns_ = std::stoull(values.at("epoch_wall_ns"));
      }
      using_state_file_ = true;
    } catch (...) {
      epoch_wall_ns_ = fallback_epoch_wall_ns_;
      epoch_mono_ns_ = fallback_epoch_mono_ns_;
      using_state_file_ = false;
    }
  }

  std::string state_file_;
  uint64_t epoch_wall_ns_ {};
  uint64_t epoch_mono_ns_ {};
  uint64_t fallback_epoch_wall_ns_ {};
  uint64_t fallback_epoch_mono_ns_ {};
  uint64_t last_reload_mono_ns_ {};
  bool using_state_file_ {false};
};

class TriggerReader
{
public:
  explicit TriggerReader(const std::string & path)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("open trigger " + path + " failed: " + std::strerror(errno));
    }
  }

  ~TriggerReader()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  int fd() const { return fd_; }

  void drain()
  {
    while (true) {
      TriggerEvent ev {};
      const ssize_t n = ::read(fd_, &ev, sizeof(ev));
      if (n == static_cast<ssize_t>(sizeof(ev))) {
        if (ev.counter < 0) {
          if (ev.counter == -1) {
            events_.clear();
          }
          continue;
        }
        events_[ev.counter] = ev.timestamp_ns;
        while (events_.size() > 256) {
          events_.erase(events_.begin());
        }
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      } else if (n < 0) {
        throw std::runtime_error(std::string("read trigger failed: ") + std::strerror(errno));
      } else {
        break;
      }
    }
  }

  bool timestamp_for_counter(long long counter, uint64_t * timestamp_ns) const
  {
    const auto it = events_.find(counter);
    if (it == events_.end()) {
      return false;
    }
    *timestamp_ns = it->second;
    return true;
  }

  bool latest_before_or_equal(uint64_t mono_ns, long long * counter, uint64_t * timestamp_ns) const
  {
    for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
      if (it->second <= mono_ns) {
        *counter = it->first;
        *timestamp_ns = it->second;
        return true;
      }
    }
    return false;
  }

private:
  int fd_ {-1};
  std::map<long long, uint64_t> events_;
};

struct MmapBuffer
{
  void * start {nullptr};
  size_t length {0};
};

struct Frame
{
  uint32_t sequence {};
  uint64_t fallback_mono_ns {};
  const uint8_t * data {nullptr};
  size_t size {};
};

// Writes the captured V4L2 payload without opening the camera a second time.
// The CSV sidecar makes every raw frame addressable by its ROS timestamp.
class CameraRecorder : public ImageRecorder
{
public:
  CameraRecorder(
    const std::string & root, const std::string & camera_name, int width, int height,
    int stride, const std::string & encoding, int every_n, double rate_hz)
  : every_n_(std::max(1, every_n))
  {
    if (root.empty()) {
      return;
    }
    const auto directory = std::filesystem::path(root);
    std::filesystem::create_directories(directory);
    stream_path_ = directory / (camera_name + ".nv12");
    index_path_ = directory / (camera_name + "_frames.csv");
    stream_.open(stream_path_, std::ios::binary | std::ios::trunc);
    index_.open(index_path_, std::ios::out | std::ios::trunc);
    if (!stream_ || !index_) {
      throw std::runtime_error("open camera recording files under " + root + " failed");
    }
    index_ << "frame_index,trigger_counter,stamp_ns,exposure_lines,offset_bytes,size_bytes,"
              "width,height,stride,encoding\n";
    index_ << std::setprecision(19);
    width_ = width;
    height_ = height;
    stride_ = stride;
    encoding_ = encoding;
    if (!std::isfinite(rate_hz) || rate_hz < 0.0) {
      throw std::runtime_error("image_record_rate_hz must be finite and non-negative");
    }
    rate_hz_ = rate_hz;
    enabled_ = true;
  }

  ~CameraRecorder()
  {
    close();
  }

  bool enabled() const override { return enabled_; }

  void write(
    uint64_t frame_index, long long trigger_counter, const rclcpp::Time & stamp,
    int64_t exposure_lines, const uint8_t * data, size_t size) override
  {
    if (!enabled_ || data == nullptr || size == 0 || (frame_index % every_n_) != 0) {
      return;
    }
    const int64_t stamp_ns = stamp.nanoseconds();
    if (rate_hz_ > 0.0) {
      if (last_seen_stamp_ns_ >= 0 && stamp_ns > last_seen_stamp_ns_) {
        rate_credit_ = std::min(
          2.0, rate_credit_ + static_cast<double>(stamp_ns - last_seen_stamp_ns_) *
          rate_hz_ / 1.0e9);
      }
      last_seen_stamp_ns_ = stamp_ns;
      if (written_ > 0 && rate_credit_ < 1.0) {
        return;
      }
      if (written_ > 0) {
        rate_credit_ -= 1.0;
      }
    }
    const auto position = stream_.tellp();
    if (position < 0) {
      disable();
      return;
    }
    stream_.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
    if (!stream_) {
      disable();
      return;
    }
    index_ << frame_index << ',' << trigger_counter << ',' << stamp.nanoseconds() << ','
           << exposure_lines << ',' << static_cast<uint64_t>(position) << ',' << size << ','
           << width_ << ',' << height_ << ',' << stride_ << ',' << encoding_ << '\n';
    if (!index_) {
      disable();
      return;
    }
    if ((++written_ % 10) == 0) {
      stream_.flush();
      index_.flush();
    }
  }

  void close() override
  {
    if (!enabled_ && !stream_.is_open() && !index_.is_open()) {
      return;
    }
    stream_.flush();
    index_.flush();
    stream_.close();
    index_.close();
    enabled_ = false;
  }

private:
  void disable()
  {
    stream_.setstate(std::ios::failbit);
    index_.setstate(std::ios::failbit);
    enabled_ = false;
  }

  bool enabled_ {false};
  int every_n_ {1};
  int width_ {};
  int height_ {};
  int stride_ {};
  std::string encoding_;
  double rate_hz_ {0.0};
  std::filesystem::path stream_path_;
  std::filesystem::path index_path_;
  std::ofstream stream_;
  std::ofstream index_;
  uint64_t written_ {0};
  int64_t last_seen_stamp_ns_ {-1};
  double rate_credit_ {0.0};
};

struct ExposureSample
{
  bool valid {false};
  int64_t exposure_lines {};
  int64_t analogue_gain {};
  uint64_t exposure_ns {};
  uint64_t center_offset_ns {};
};

class V4L2ControlReader
{
public:
  explicit V4L2ControlReader(const std::string & path)
  : path_(path)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("open control device " + path + " failed: " + std::strerror(errno));
    }
  }

  ~V4L2ControlReader()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool get_int(uint32_t id, int64_t * value) const
  {
    v4l2_control control {};
    control.id = id;
    if (::ioctl(fd_, VIDIOC_G_CTRL, &control) < 0) {
      return false;
    }
    *value = control.value;
    return true;
  }

  bool set_int(uint32_t id, int64_t value) const
  {
    v4l2_control control {};
    control.id = id;
    control.value = static_cast<int32_t>(value);
    return ::ioctl(fd_, VIDIOC_S_CTRL, &control) == 0;
  }

  bool get_int64(uint32_t id, int64_t * value) const
  {
    v4l2_ext_control control {};
    control.id = id;
    v4l2_ext_controls controls {};
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(id);
    controls.count = 1;
    controls.controls = &control;
    if (::ioctl(fd_, VIDIOC_G_EXT_CTRLS, &controls) < 0) {
      return false;
    }
    *value = control.value64;
    return true;
  }

  const std::string & path() const { return path_; }

private:
  std::string path_;
  int fd_ {-1};
};

class ExposureTiming
{
public:
  ExposureTiming(const std::string & subdev, int active_width, bool enabled)
  : enabled_(enabled), active_width_(active_width)
  {
    if (!enabled_) {
      return;
    }
    if (subdev.empty()) {
      throw std::runtime_error("exposure center offset enabled but subdev path is empty");
    }

    controls_ = std::make_unique<V4L2ControlReader>(subdev);
    int64_t hblank = 0;
    int64_t pixel_rate = 0;
    if (!controls_->get_int(V4L2_CID_HBLANK, &hblank)) {
      throw std::runtime_error("read " + subdev + " horizontal_blanking failed");
    }
    if (!controls_->get_int64(V4L2_CID_PIXEL_RATE, &pixel_rate)) {
      throw std::runtime_error("read " + subdev + " pixel_rate failed");
    }
    if (pixel_rate <= 0 || active_width_ <= 0 || hblank < 0) {
      throw std::runtime_error("invalid exposure timing controls from " + subdev);
    }

    hblank_ = hblank;
    pixel_rate_ = pixel_rate;
    valid_timing_ = true;
  }

  ExposureSample sample()
  {
    refresh_if_needed();
    return last_sample_;
  }

  bool enabled() const { return enabled_ && valid_timing_; }
  int64_t hblank() const { return hblank_; }
  int64_t pixel_rate() const { return pixel_rate_; }
  uint64_t read_failures() const { return read_failures_; }
  const ExposureSample & last_sample() const { return last_sample_; }
  const std::string & path() const { return controls_->path(); }

  bool set_vertical_blanking(int64_t value)
  {
    return enabled_ && valid_timing_ && controls_ && controls_->set_int(V4L2_CID_VBLANK, value);
  }

private:
  void refresh_if_needed()
  {
    if (!enabled_ || !valid_timing_ || !controls_) {
      return;
    }

    const uint64_t now_mono_ns = clock_ns(CLOCK_MONOTONIC);
    if (last_refresh_mono_ns_ != 0 &&
      now_mono_ns - last_refresh_mono_ns_ < kExposureRefreshIntervalNs)
    {
      return;
    }
    last_refresh_mono_ns_ = now_mono_ns;

    int64_t exposure_lines = 0;
    int64_t analogue_gain = 0;
    if (!controls_->get_int(V4L2_CID_EXPOSURE, &exposure_lines)) {
      ++read_failures_;
      return;
    }
    controls_->get_int(V4L2_CID_ANALOGUE_GAIN, &analogue_gain);

    const long double line_length = static_cast<long double>(active_width_ + hblank_);
    const long double exposure_ns =
      static_cast<long double>(exposure_lines) * line_length * 1000000000.0L /
      static_cast<long double>(pixel_rate_);

    last_sample_.valid = true;
    last_sample_.exposure_lines = exposure_lines;
    last_sample_.analogue_gain = analogue_gain;
    last_sample_.exposure_ns = static_cast<uint64_t>(exposure_ns + 0.5L);
    last_sample_.center_offset_ns = last_sample_.exposure_ns / 2;
  }

  bool enabled_ {false};
  bool valid_timing_ {false};
  int active_width_ {};
  int64_t hblank_ {};
  int64_t pixel_rate_ {};
  uint64_t read_failures_ {};
  uint64_t last_refresh_mono_ns_ {};
  ExposureSample last_sample_ {};
  std::unique_ptr<V4L2ControlReader> controls_;
};

class V4L2Camera
{
public:
  explicit V4L2Camera(const std::string & path)
  : path_(path)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("open video " + path + " failed: " + std::strerror(errno));
    }

    v4l2_capability cap {};
    xioctl(VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP");
    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
      type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
      type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
      throw std::runtime_error(path + " is not a capture device");
    }

    v4l2_format fmt {};
    fmt.type = type_;
    xioctl(VIDIOC_G_FMT, &fmt, "VIDIOC_G_FMT");
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      width_ = fmt.fmt.pix_mp.width;
      height_ = fmt.fmt.pix_mp.height;
      bytes_per_line_ = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
      image_size_ = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    } else {
      width_ = fmt.fmt.pix.width;
      height_ = fmt.fmt.pix.height;
      bytes_per_line_ = fmt.fmt.pix.bytesperline;
      image_size_ = fmt.fmt.pix.sizeimage;
    }
  }

  ~V4L2Camera()
  {
    try {
      stop();
    } catch (...) {
    }
    for (auto & buffer : buffers_) {
      if (buffer.start && buffer.start != MAP_FAILED) {
        ::munmap(buffer.start, buffer.length);
      }
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  int fd() const { return fd_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int bytes_per_line() const { return bytes_per_line_; }
  const std::string & path() const { return path_; }

  void setup_buffers()
  {
    v4l2_requestbuffers req {};
    req.count = kBufferCount;
    req.type = type_;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");
    if (req.count < 2) {
      throw std::runtime_error(path_ + " returned too few mmap buffers");
    }

    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
      v4l2_buffer buf {};
      v4l2_plane planes[1] {};
      buf.type = type_;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = 1;
      }
      xioctl(VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");

      const size_t length =
        type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ? planes[0].length : buf.length;
      const off_t offset =
        type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ? planes[0].m.mem_offset : buf.m.offset;

      void * start = ::mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, offset);
      if (start == MAP_FAILED) {
        throw std::runtime_error(path_ + " mmap failed: " + std::strerror(errno));
      }
      buffers_[i] = {start, length};
    }

    queue_all();
  }

  void queue_all()
  {
    for (uint32_t i = 0; i < buffers_.size(); ++i) {
      qbuf(i);
    }
  }

  void start()
  {
    if (!streaming_) {
      xioctl(VIDIOC_STREAMON, &type_, "VIDIOC_STREAMON");
      streaming_ = true;
    }
  }

  void stop()
  {
    if (streaming_) {
      ::ioctl(fd_, VIDIOC_STREAMOFF, &type_);
      streaming_ = false;
    }
  }

  bool dequeue(Frame * frame, uint32_t * index)
  {
    v4l2_buffer buf {};
    v4l2_plane planes[1] {};
    buf.type = type_;
    buf.memory = V4L2_MEMORY_MMAP;
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      buf.m.planes = planes;
      buf.length = 1;
    }

    const int ret = ::ioctl(fd_, VIDIOC_DQBUF, &buf);
    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    if (ret < 0) {
      throw std::runtime_error(path_ + " VIDIOC_DQBUF failed: " + std::strerror(errno));
    }

    const size_t bytesused =
      type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ? planes[0].bytesused : buf.bytesused;
    const uint64_t fallback_mono_ns = timeval_to_ns(buf.timestamp);
    *frame = {
      buf.sequence,
      fallback_mono_ns,
      static_cast<const uint8_t *>(buffers_.at(buf.index).start),
      bytesused,
    };
    *index = buf.index;
    return true;
  }

  void requeue(uint32_t index)
  {
    qbuf(index);
  }

private:
  void qbuf(uint32_t index)
  {
    v4l2_buffer buf {};
    v4l2_plane planes[1] {};
    buf.type = type_;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      buf.m.planes = planes;
      buf.length = 1;
    }
    xioctl(VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
  }

  void xioctl(unsigned long request, void * arg, const char * name)
  {
    if (::ioctl(fd_, request, arg) < 0) {
      throw std::runtime_error(path_ + " " + name + " failed: " + std::strerror(errno));
    }
  }

  std::string path_;
  int fd_ {-1};
  v4l2_buf_type type_ {};
  int width_ {};
  int height_ {};
  int bytes_per_line_ {};
  size_t image_size_ {};
  bool streaming_ {false};
  std::vector<MmapBuffer> buffers_;
};

class Imx586DualCameraNode : public rclcpp::Node
{
public:
  Imx586DualCameraNode()
  : Node("imx586_dual_camera")
  {
    trigger_dev_ = declare_parameter<std::string>("trigger_dev", "/dev/imx586_trigger");
    trigger_enable_ = declare_parameter<std::string>(
      "trigger_enable", "/sys/kernel/imx586_trigger/enable");
    video_dev1_ = declare_parameter<std::string>("video_dev1", "/dev/video44");
    video_dev2_ = declare_parameter<std::string>("video_dev2", "/dev/video53");
    video_dev3_ = declare_parameter<std::string>("video_dev3", "/dev/video62");
    video_dev4_ = declare_parameter<std::string>("video_dev4", "/dev/video71");
    subdev1_ = declare_parameter<std::string>("subdev1", "/dev/v4l-subdev2");
    subdev2_ = declare_parameter<std::string>("subdev2", "/dev/v4l-subdev7");
    subdev3_ = declare_parameter<std::string>("subdev3", "/dev/v4l-subdev12");
    subdev4_ = declare_parameter<std::string>("subdev4", "/dev/v4l-subdev17");
    topic1_ = declare_parameter<std::string>("topic1", "/imx586/cam1/image_raw");
    topic2_ = declare_parameter<std::string>("topic2", "/imx586/cam2/image_raw");
    topic3_ = declare_parameter<std::string>("topic3", "/imx586/cam3/image_raw");
    topic4_ = declare_parameter<std::string>("topic4", "/imx586/cam4/image_raw");
    header_topic1_ = declare_parameter<std::string>("header_topic1", "/imx586/cam1/header");
    header_topic2_ = declare_parameter<std::string>("header_topic2", "/imx586/cam2/header");
    header_topic3_ = declare_parameter<std::string>("header_topic3", "/imx586/cam3/header");
    header_topic4_ = declare_parameter<std::string>("header_topic4", "/imx586/cam4/header");
    preview_topic1_ = declare_parameter<std::string>("preview_topic1", "");
    preview_width_ = declare_parameter<int>("preview_width", 1200);
    preview_every_n_frames_ = declare_parameter<int>("preview_every_n_frames", 10);
    horizontal_flip1_ = declare_parameter<bool>("horizontal_flip1", false);
    horizontal_flip2_ = declare_parameter<bool>("horizontal_flip2", false);
    horizontal_flip3_ = declare_parameter<bool>("horizontal_flip3", false);
    horizontal_flip4_ = declare_parameter<bool>("horizontal_flip4", false);
    frame_id1_ = declare_parameter<std::string>("frame_id1", "imx586_cam1_optical");
    frame_id2_ = declare_parameter<std::string>("frame_id2", "imx586_cam2_optical");
    frame_id3_ = declare_parameter<std::string>("frame_id3", "imx586_cam3_optical");
    frame_id4_ = declare_parameter<std::string>("frame_id4", "imx586_cam4_optical");
    state_file_ = declare_parameter<std::string>("state_file", "/tmp/vanjee_sync_state");
    encoding_ = declare_parameter<std::string>("encoding", "nv12");
    image_record_dir_ = declare_parameter<std::string>("image_record_dir", "");
    image_record_every_n_ = declare_parameter<int>("image_record_every_n", 1);
    image_record_rate_hz_ = declare_parameter<double>("image_record_rate_hz", 3.0);
    image_record_codec_ = declare_parameter<std::string>("image_record_codec", "h264");
    sensor_vertical_blanking_ = declare_parameter<int>("sensor_vertical_blanking", 6192);
    queue_size_ = declare_parameter<int>("queue_size", 4);
    enable_cam2_ = declare_parameter<bool>("enable_cam2", true);
    enable_cam3_ = declare_parameter<bool>("enable_cam3", true);
    enable_cam4_ = declare_parameter<bool>("enable_cam4", true);
    control_trigger_enable_ = declare_parameter<bool>("control_trigger_enable", true);
    apply_exposure_center_offset_ =
      declare_parameter<bool>("apply_exposure_center_offset", true);

    pub1_ = create_publisher<sensor_msgs::msg::Image>(topic1_, rclcpp::SensorDataQoS());
    header_pub1_ = create_publisher<std_msgs::msg::Header>(header_topic1_, rclcpp::SensorDataQoS());
    if (!preview_topic1_.empty()) {
      if (preview_width_ <= 0 || preview_every_n_frames_ <= 0) {
        throw std::runtime_error("preview_width and preview_every_n_frames must be positive");
      }
      preview_pub1_ =
        create_publisher<sensor_msgs::msg::Image>(preview_topic1_, rclcpp::SensorDataQoS());
    }
    if (enable_cam2_) {
      pub2_ = create_publisher<sensor_msgs::msg::Image>(topic2_, rclcpp::SensorDataQoS());
      header_pub2_ =
        create_publisher<std_msgs::msg::Header>(header_topic2_, rclcpp::SensorDataQoS());
    }
    if (enable_cam3_) {
      pub3_ = create_publisher<sensor_msgs::msg::Image>(topic3_, rclcpp::SensorDataQoS());
      header_pub3_ =
        create_publisher<std_msgs::msg::Header>(header_topic3_, rclcpp::SensorDataQoS());
    }
    if (enable_cam4_) {
      pub4_ = create_publisher<sensor_msgs::msg::Image>(topic4_, rclcpp::SensorDataQoS());
      header_pub4_ =
        create_publisher<std_msgs::msg::Header>(header_topic4_, rclcpp::SensorDataQoS());
    }
  }

  int run()
  {
    TimeMapper time_mapper(state_file_);
    TriggerReader trigger(trigger_dev_);
    V4L2Camera cam1(video_dev1_);
    ExposureTiming exposure1(subdev1_, cam1.width(), apply_exposure_center_offset_);
    std::unique_ptr<V4L2Camera> cam2;
    std::unique_ptr<V4L2Camera> cam3;
    std::unique_ptr<V4L2Camera> cam4;
    std::unique_ptr<ExposureTiming> exposure2;
    std::unique_ptr<ExposureTiming> exposure3;
    std::unique_ptr<ExposureTiming> exposure4;
    if (enable_cam2_) {
      cam2 = std::make_unique<V4L2Camera>(video_dev2_);
      exposure2 = std::make_unique<ExposureTiming>(
        subdev2_, cam2->width(), apply_exposure_center_offset_);
    }
    if (enable_cam3_) {
      cam3 = std::make_unique<V4L2Camera>(video_dev3_);
      exposure3 = std::make_unique<ExposureTiming>(
        subdev3_, cam3->width(), apply_exposure_center_offset_);
    }
    if (enable_cam4_) {
      cam4 = std::make_unique<V4L2Camera>(video_dev4_);
      exposure4 = std::make_unique<ExposureTiming>(
        subdev4_, cam4->width(), apply_exposure_center_offset_);
    }
    std::unique_ptr<ImageRecorder> recorder1;
    std::unique_ptr<ImageRecorder> recorder2;
    std::unique_ptr<ImageRecorder> recorder3;
    std::unique_ptr<ImageRecorder> recorder4;
    if (!image_record_dir_.empty()) {
      if (image_record_every_n_ <= 0) {
        throw std::runtime_error("image_record_every_n must be positive");
      }
      auto make_recorder = [&](const std::string & name, int width, int height, int stride) {
        if (image_record_codec_ == "h264") {
          return std::unique_ptr<ImageRecorder>(std::make_unique<H264CameraRecorder>(
                   image_record_dir_, name, width, height, stride,
                   image_record_every_n_, image_record_rate_hz_));
        }
        if (image_record_codec_ == "raw") {
          return std::unique_ptr<ImageRecorder>(std::make_unique<CameraRecorder>(
                   image_record_dir_, name, width, height, stride, encoding_,
                   image_record_every_n_, image_record_rate_hz_));
        }
        throw std::runtime_error("image_record_codec must be h264 or raw");
      };
      recorder1 = make_recorder("cam1", cam1.width(), cam1.height(), cam1.bytes_per_line());
      if (cam2) {
        recorder2 = make_recorder("cam2", cam2->width(), cam2->height(), cam2->bytes_per_line());
      }
      if (cam3) {
        recorder3 = make_recorder("cam3", cam3->width(), cam3->height(), cam3->bytes_per_line());
      }
      if (cam4) {
        recorder4 = make_recorder("cam4", cam4->width(), cam4->height(), cam4->bytes_per_line());
      }
      RCLCPP_INFO(
        get_logger(),
        "external image recording enabled: dir=%s rate=%.3f Hz every_n=%d codec=%s format=%s",
        image_record_dir_.c_str(), image_record_rate_hz_, image_record_every_n_,
        image_record_codec_.c_str(), encoding_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "external image recording disabled");
    }

    RCLCPP_INFO(
      get_logger(), "cam1=%s %dx%d stride=%d topic=%s",
      cam1.path().c_str(), cam1.width(), cam1.height(), cam1.bytes_per_line(), topic1_.c_str());
    RCLCPP_INFO(
      get_logger(), "cam1 software horizontal flip=%s",
      horizontal_flip1_ ? "enabled" : "disabled");
    if (preview_pub1_) {
      RCLCPP_INFO(
        get_logger(), "cam1 preview topic=%s width=%d every_n_frames=%d encoding=mono8",
        preview_topic1_.c_str(), preview_width_, preview_every_n_frames_);
    }
    if (cam2) {
      RCLCPP_INFO(
        get_logger(), "cam2=%s %dx%d stride=%d topic=%s",
        cam2->path().c_str(), cam2->width(), cam2->height(), cam2->bytes_per_line(),
        topic2_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "cam2 disabled");
    }
    if (cam3) {
      RCLCPP_INFO(
        get_logger(), "cam3=%s %dx%d stride=%d topic=%s",
        cam3->path().c_str(), cam3->width(), cam3->height(), cam3->bytes_per_line(),
        topic3_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "cam3 disabled");
    }
    if (cam4) {
      RCLCPP_INFO(
        get_logger(), "cam4=%s %dx%d stride=%d topic=%s",
        cam4->path().c_str(), cam4->width(), cam4->height(), cam4->bytes_per_line(),
        topic4_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "cam4 disabled");
    }
    RCLCPP_INFO(
      get_logger(), "time mapping: %s",
      time_mapper.using_state_file() ? "state_file" : "system realtime/monotonic fallback");
    if (apply_exposure_center_offset_) {
      const auto sample1 = exposure1.sample();
      RCLCPP_INFO(
        get_logger(),
        "exposure center offset enabled: cam1_subdev=%s line_length=%ld pixel_rate=%ld "
        "initial_exposure=%ld lines offset=%.3f ms",
        exposure1.path().c_str(), cam1.width() + exposure1.hblank(), exposure1.pixel_rate(),
        sample1.exposure_lines, sample1.center_offset_ns / 1000000.0);
      if (exposure2) {
        const auto sample2 = exposure2->sample();
        RCLCPP_INFO(
          get_logger(),
          "cam2 exposure: subdev=%s line_length=%ld pixel_rate=%ld "
          "initial_exposure=%ld lines offset=%.3f ms",
          exposure2->path().c_str(), cam2->width() + exposure2->hblank(), exposure2->pixel_rate(),
          sample2.exposure_lines, sample2.center_offset_ns / 1000000.0);
      }
      if (exposure3) {
        const auto sample3 = exposure3->sample();
        RCLCPP_INFO(
          get_logger(),
          "cam3 exposure: subdev=%s line_length=%ld pixel_rate=%ld "
          "initial_exposure=%ld lines offset=%.3f ms",
          exposure3->path().c_str(), cam3->width() + exposure3->hblank(), exposure3->pixel_rate(),
          sample3.exposure_lines, sample3.center_offset_ns / 1000000.0);
      }
      if (exposure4) {
        const auto sample4 = exposure4->sample();
        RCLCPP_INFO(
          get_logger(),
          "cam4 exposure: subdev=%s line_length=%ld pixel_rate=%ld "
          "initial_exposure=%ld lines offset=%.3f ms",
          exposure4->path().c_str(), cam4->width() + exposure4->hblank(), exposure4->pixel_rate(),
          sample4.exposure_lines, sample4.center_offset_ns / 1000000.0);
      }
      RCLCPP_INFO(get_logger(), "exposure queries are cached and refreshed at 5 Hz");
    } else {
      RCLCPP_INFO(get_logger(), "exposure center offset disabled");
    }

    if (control_trigger_enable_) {
      write_text_file(trigger_enable_, "0");
    }
    trigger.drain();

    cam1.setup_buffers();
    if (cam2) {
      cam2->setup_buffers();
    }
    if (cam3) {
      cam3->setup_buffers();
    }
    if (cam4) {
      cam4->setup_buffers();
    }
    cam1.start();
    if (cam2) {
      cam2->start();
    }
    if (cam3) {
      cam3->start();
    }
    if (cam4) {
      cam4->start();
    }

    if (sensor_vertical_blanking_ > 0) {
      auto apply_vertical_blanking = [&](ExposureTiming * exposure, const char * name) {
        if (exposure != nullptr && !exposure->set_vertical_blanking(sensor_vertical_blanking_)) {
          RCLCPP_WARN(
            get_logger(), "failed to set %s vertical_blanking=%d; sensor may keep its default rate",
            name, sensor_vertical_blanking_);
        }
      };
      apply_vertical_blanking(&exposure1, "cam1");
      apply_vertical_blanking(exposure2.get(), "cam2");
      apply_vertical_blanking(exposure3.get(), "cam3");
      apply_vertical_blanking(exposure4.get(), "cam4");
      RCLCPP_INFO(
        get_logger(), "sensor vertical blanking requested: %d (applied after STREAMON)",
        sensor_vertical_blanking_);
    }

    if (control_trigger_enable_) {
      write_text_file(trigger_enable_, "1");
    }
    bool cleanup_needed = true;
    auto cleanup = [&]() {
      if (!cleanup_needed) {
        return;
      }
      if (control_trigger_enable_) {
        try {
          write_text_file(trigger_enable_, "0");
        } catch (const std::exception & exc) {
          RCLCPP_WARN(get_logger(), "failed to disable trigger: %s", exc.what());
        }
      }
      cam1.stop();
      if (cam2) {
        cam2->stop();
      }
      if (cam3) {
        cam3->stop();
      }
      if (cam4) {
        cam4->stop();
      }
      cleanup_needed = false;
    };

    uint64_t frames1 = 0;
    uint64_t frames2 = 0;
    uint64_t frames3 = 0;
    uint64_t frames4 = 0;
    uint64_t missing_trigger1 = 0;
    uint64_t missing_trigger2 = 0;
    uint64_t missing_trigger3 = 0;
    uint64_t missing_trigger4 = 0;
    uint64_t duplicate_trigger1 = 0;
    uint64_t duplicate_trigger2 = 0;
    uint64_t duplicate_trigger3 = 0;
    uint64_t duplicate_trigger4 = 0;
    long long base_counter1 = std::numeric_limits<long long>::min();
    long long base_counter2 = std::numeric_limits<long long>::min();
    long long base_counter3 = std::numeric_limits<long long>::min();
    long long base_counter4 = std::numeric_limits<long long>::min();
    long long last_counter1 = std::numeric_limits<long long>::min();
    long long last_counter2 = std::numeric_limits<long long>::min();
    long long last_counter3 = std::numeric_limits<long long>::min();
    long long last_counter4 = std::numeric_limits<long long>::min();
    auto last_log = now();

    try {
      while (rclcpp::ok()) {
        time_mapper.maybe_reload();

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(trigger.fd(), &fds);
        FD_SET(cam1.fd(), &fds);
        int max_fd = std::max(trigger.fd(), cam1.fd());
        if (cam2) {
          FD_SET(cam2->fd(), &fds);
          max_fd = std::max(max_fd, cam2->fd());
        }
        if (cam3) {
          FD_SET(cam3->fd(), &fds);
          max_fd = std::max(max_fd, cam3->fd());
        }
        if (cam4) {
          FD_SET(cam4->fd(), &fds);
          max_fd = std::max(max_fd, cam4->fd());
        }
        timeval tv {1, 0};
        const int ret = ::select(max_fd + 1, &fds, nullptr, nullptr, &tv);
        if (ret < 0) {
          if (errno == EINTR) {
            continue;
          }
          throw std::runtime_error(std::string("select failed: ") + std::strerror(errno));
        }

        if (ret > 0 && FD_ISSET(trigger.fd(), &fds)) {
          trigger.drain();
        }
        trigger.drain();

        if (ret > 0 && FD_ISSET(cam1.fd(), &fds)) {
          handle_camera(
            cam1, trigger, time_mapper, pub1_, header_pub1_, preview_pub1_, frame_id1_,
            horizontal_flip1_, &frames1, &missing_trigger1, &duplicate_trigger1,
            &base_counter1, &last_counter1, &exposure1, recorder1.get());
        }
        if (cam2 && ret > 0 && FD_ISSET(cam2->fd(), &fds)) {
          handle_camera(
            *cam2, trigger, time_mapper, pub2_, header_pub2_, nullptr, frame_id2_,
            horizontal_flip2_, &frames2, &missing_trigger2, &duplicate_trigger2,
            &base_counter2, &last_counter2, exposure2.get(), recorder2.get());
        }
        if (cam3 && ret > 0 && FD_ISSET(cam3->fd(), &fds)) {
          handle_camera(
            *cam3, trigger, time_mapper, pub3_, header_pub3_, nullptr, frame_id3_,
            horizontal_flip3_, &frames3, &missing_trigger3, &duplicate_trigger3,
            &base_counter3, &last_counter3, exposure3.get(), recorder3.get());
        }
        if (cam4 && ret > 0 && FD_ISSET(cam4->fd(), &fds)) {
          handle_camera(
            *cam4, trigger, time_mapper, pub4_, header_pub4_, nullptr, frame_id4_,
            horizontal_flip4_, &frames4, &missing_trigger4, &duplicate_trigger4,
            &base_counter4, &last_counter4, exposure4.get(), recorder4.get());
        }

        const auto now_ros = now();
        if ((now_ros - last_log).seconds() >= 5.0) {
          if (exposure2) {
            RCLCPP_INFO(
              get_logger(),
              "published cam1=%lu cam2=%lu missing_trigger cam1=%lu cam2=%lu "
              "duplicate_trigger cam1=%lu cam2=%lu "
              "exposure cam1=%ld lines offset=%.3f ms cam2=%ld lines offset=%.3f ms",
              frames1, frames2, missing_trigger1, missing_trigger2,
              duplicate_trigger1, duplicate_trigger2,
              exposure1.last_sample().exposure_lines,
              exposure1.last_sample().center_offset_ns / 1000000.0,
              exposure2->last_sample().exposure_lines,
              exposure2->last_sample().center_offset_ns / 1000000.0);
          } else {
            RCLCPP_INFO(
              get_logger(),
              "published cam1=%lu missing_trigger=%lu duplicate_trigger=%lu "
              "exposure=%ld lines offset=%.3f ms",
              frames1, missing_trigger1, duplicate_trigger1,
              exposure1.last_sample().exposure_lines,
              exposure1.last_sample().center_offset_ns / 1000000.0);
          }
          if (cam3) {
            RCLCPP_INFO(
              get_logger(),
              "published cam3=%lu missing_trigger=%lu duplicate_trigger=%lu "
              "exposure=%ld lines offset=%.3f ms",
              frames3, missing_trigger3, duplicate_trigger3,
              exposure3->last_sample().exposure_lines,
              exposure3->last_sample().center_offset_ns / 1000000.0);
          }
          if (cam4) {
            RCLCPP_INFO(
              get_logger(),
              "published cam4=%lu missing_trigger=%lu duplicate_trigger=%lu "
              "exposure=%ld lines offset=%.3f ms",
              frames4, missing_trigger4, duplicate_trigger4,
              exposure4->last_sample().exposure_lines,
              exposure4->last_sample().center_offset_ns / 1000000.0);
          }
          last_log = now_ros;
        }
      }
    } catch (...) {
      cleanup();
      throw;
    }

    cleanup();
    if (cam2) {
      RCLCPP_INFO(
        get_logger(),
        "stopped: cam1=%lu cam2=%lu missing_trigger cam1=%lu cam2=%lu "
        "duplicate_trigger cam1=%lu cam2=%lu",
        frames1, frames2, missing_trigger1, missing_trigger2,
        duplicate_trigger1, duplicate_trigger2);
    } else {
      RCLCPP_INFO(
        get_logger(), "stopped: cam1=%lu missing_trigger=%lu duplicate_trigger=%lu",
        frames1, missing_trigger1, duplicate_trigger1);
    }
    if (cam3) {
      RCLCPP_INFO(
        get_logger(), "stopped: cam3=%lu missing_trigger=%lu duplicate_trigger=%lu",
        frames3, missing_trigger3, duplicate_trigger3);
    }
    if (cam4) {
      RCLCPP_INFO(
        get_logger(), "stopped: cam4=%lu missing_trigger=%lu duplicate_trigger=%lu",
        frames4, missing_trigger4, duplicate_trigger4);
    }
    return 0;
  }

private:
  void handle_camera(
    V4L2Camera & camera,
    const TriggerReader & trigger,
    const TimeMapper & time_mapper,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
    const rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr & header_pub,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & preview_pub,
    const std::string & frame_id,
    bool horizontal_flip,
    uint64_t * frame_count,
    uint64_t * missing_trigger_count,
    uint64_t * duplicate_trigger_count,
    long long * base_counter,
    long long * last_counter,
    ExposureTiming * exposure_timing,
    ImageRecorder * recorder)
  {
    while (true) {
      Frame frame;
      uint32_t index = 0;
      if (!camera.dequeue(&frame, &index)) {
        break;
      }

      uint64_t stamp_mono_ns = 0;
      bool matched_trigger = false;
      long long matched_counter = -1;
      if (*base_counter != std::numeric_limits<long long>::min()) {
        const auto expected_counter = *base_counter + static_cast<long long>(*frame_count);
        matched_trigger = trigger.timestamp_for_counter(expected_counter, &stamp_mono_ns);
        if (matched_trigger) {
          matched_counter = expected_counter;
        }
      }

      if (!matched_trigger) {
        matched_trigger = trigger.latest_before_or_equal(
          frame.fallback_mono_ns, &matched_counter, &stamp_mono_ns);
        if (matched_trigger && *base_counter == std::numeric_limits<long long>::min()) {
          *base_counter = matched_counter - static_cast<long long>(*frame_count);
        }
      }

      if (matched_trigger &&
        *last_counter != std::numeric_limits<long long>::min() &&
        matched_counter <= *last_counter)
      {
        camera.requeue(index);
        ++(*duplicate_trigger_count);
        continue;
      }

      if (!matched_trigger) {
        stamp_mono_ns = frame.fallback_mono_ns;
        ++(*missing_trigger_count);
      }

      int64_t exposure_lines = -1;
      if (matched_trigger && exposure_timing != nullptr && exposure_timing->enabled()) {
        const auto exposure = exposure_timing->sample();
        if (exposure.valid) {
          exposure_lines = exposure.exposure_lines;
          stamp_mono_ns += exposure.center_offset_ns;
        }
      }

      auto msg = sensor_msgs::msg::Image();
      msg.header.stamp = time_mapper.stamp_from_mono_ns(stamp_mono_ns);
      msg.header.frame_id = frame_id;
      msg.height = static_cast<uint32_t>(camera.height());
      msg.width = static_cast<uint32_t>(camera.width());
      msg.encoding = encoding_;
      msg.is_bigendian = false;
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(camera.bytes_per_line());
      msg.data.resize(frame.size);
      std::memcpy(msg.data.data(), frame.data, frame.size);
      if (horizontal_flip) {
        if (encoding_ != "nv12") {
          camera.requeue(index);
          throw std::runtime_error("horizontal_flip currently requires nv12 encoding");
        }
        const size_t stride = static_cast<size_t>(camera.bytes_per_line());
        const size_t y_plane_size = stride * static_cast<size_t>(camera.height());
        const size_t required_size = y_plane_size + y_plane_size / 2;
        if (camera.width() % 2 != 0 || msg.data.size() < required_size) {
          camera.requeue(index);
          throw std::runtime_error("invalid NV12 geometry for horizontal_flip");
        }
        for (int y = 0; y < camera.height(); ++y) {
          auto begin = msg.data.begin() +
            static_cast<std::vector<uint8_t>::difference_type>(stride * y);
          std::reverse(begin, begin + camera.width());
        }
        for (int y = 0; y < camera.height() / 2; ++y) {
          uint8_t * row = msg.data.data() + y_plane_size + stride * static_cast<size_t>(y);
          for (int left = 0, right = camera.width() - 2; left < right; left += 2, right -= 2) {
            std::swap(row[left], row[right]);
            std::swap(row[left + 1], row[right + 1]);
          }
        }
      }

      if (preview_pub && (*frame_count % static_cast<uint64_t>(preview_every_n_frames_)) == 0) {
        const int preview_width = std::min(preview_width_, camera.width());
        const int preview_height =
          std::max(1, camera.height() * preview_width / camera.width());
        sensor_msgs::msg::Image preview;
        preview.header = msg.header;
        preview.height = static_cast<uint32_t>(preview_height);
        preview.width = static_cast<uint32_t>(preview_width);
        preview.encoding = "mono8";
        preview.is_bigendian = false;
        preview.step = static_cast<sensor_msgs::msg::Image::_step_type>(preview_width);
        preview.data.resize(static_cast<size_t>(preview_width) * preview_height);
        for (int y = 0; y < preview_height; ++y) {
          const int source_y = y * camera.height() / preview_height;
          const uint8_t * source_row =
            msg.data.data() + static_cast<size_t>(source_y) * camera.bytes_per_line();
          uint8_t * target_row = preview.data.data() + static_cast<size_t>(y) * preview_width;
          for (int x = 0; x < preview_width; ++x) {
            target_row[x] = source_row[x * camera.width() / preview_width];
          }
        }
        preview_pub->publish(std::move(preview));
      }

      const auto stamp = time_mapper.stamp_from_mono_ns(stamp_mono_ns);
      if (recorder != nullptr) {
        recorder->write(*frame_count, matched_counter, stamp, exposure_lines, msg.data.data(), msg.data.size());
      }
      camera.requeue(index);
      msg.header.stamp = stamp;
      header_pub->publish(msg.header);
      pub->publish(std::move(msg));
      if (matched_trigger) {
        *last_counter = matched_counter;
      }
      ++(*frame_count);
    }
  }

  std::string trigger_dev_;
  std::string trigger_enable_;
  std::string video_dev1_;
  std::string video_dev2_;
  std::string video_dev3_;
  std::string video_dev4_;
  std::string subdev1_;
  std::string subdev2_;
  std::string subdev3_;
  std::string subdev4_;
  std::string topic1_;
  std::string topic2_;
  std::string topic3_;
  std::string topic4_;
  std::string header_topic1_;
  std::string header_topic2_;
  std::string header_topic3_;
  std::string header_topic4_;
  std::string preview_topic1_;
  std::string frame_id1_;
  std::string frame_id2_;
  std::string frame_id3_;
  std::string frame_id4_;
  std::string state_file_;
  std::string encoding_;
  std::string image_record_dir_;
  int queue_size_ {};
  int preview_width_ {};
  int preview_every_n_frames_ {};
  int image_record_every_n_ {1};
  double image_record_rate_hz_ {3.0};
  std::string image_record_codec_ {"h264"};
  int sensor_vertical_blanking_ {6192};
  bool enable_cam2_ {};
  bool enable_cam3_ {};
  bool enable_cam4_ {};
  bool horizontal_flip1_ {};
  bool horizontal_flip2_ {};
  bool horizontal_flip3_ {};
  bool horizontal_flip4_ {};
  bool control_trigger_enable_ {};
  bool apply_exposure_center_offset_ {};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub1_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub3_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub4_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub3_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub4_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub1_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Imx586DualCameraNode>();
    const int ret = node->run();
    rclcpp::shutdown();
    return ret;
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "imx586_dual_camera_node error: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }
}
