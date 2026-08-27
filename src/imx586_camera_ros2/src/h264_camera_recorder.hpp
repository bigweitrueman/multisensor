#pragma once

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/time.hpp>

class ImageRecorder
{
public:
  virtual ~ImageRecorder() = default;
  virtual bool enabled() const = 0;
  virtual void write(
    uint64_t frame_index, long long trigger_counter, const rclcpp::Time & stamp,
    int64_t exposure_lines, const uint8_t * data, size_t size) = 0;
  virtual void close() = 0;
};

// Asynchronous all-intra H.264 recorder. The V4L2 capture thread only copies
// selected frames into a bounded queue; MPP encoding and storage happen here.
class H264CameraRecorder : public ImageRecorder
{
public:
  H264CameraRecorder(
    const std::string & root, const std::string & camera_name, int width, int height,
    int stride, int every_n, double rate_hz)
  : every_n_(std::max(1, every_n)), width_(width), height_(height), stride_(stride),
    rate_hz_(rate_hz)
  {
    if (root.empty()) {
      return;
    }
    if (width_ <= 0 || height_ <= 0 || stride_ < width_ * 2) {
      throw std::runtime_error("invalid UYVY geometry for H.264 recorder");
    }
    if (!std::isfinite(rate_hz_) || rate_hz_ < 0.0) {
      throw std::runtime_error("image_record_rate_hz must be finite and non-negative");
    }

    const auto directory = std::filesystem::path(root);
    std::filesystem::create_directories(directory);
    stream_path_ = directory / (camera_name + ".h264");
    index_path_ = directory / (camera_name + "_frames.csv");
    stream_.open(stream_path_, std::ios::binary | std::ios::trunc);
    index_.open(index_path_, std::ios::out | std::ios::trunc);
    if (!stream_ || !index_) {
      throw std::runtime_error("open H.264 camera recording files under " + root + " failed");
    }
    index_ << "frame_index,trigger_counter,stamp_ns,exposure_lines,offset_bytes,size_bytes,"
              "width,height,stride,encoding\n";
    index_ << std::setprecision(19);

    std::call_once(gst_once_, []() {
      int argc = 0;
      char ** argv = nullptr;
      gst_init(&argc, &argv);
    });
    create_pipeline();
    enabled_.store(true);
    worker_ = std::thread(&H264CameraRecorder::worker_loop, this);
  }

  ~H264CameraRecorder() { close(); }

  bool enabled() const override { return enabled_.load(); }

  void write(
    uint64_t frame_index, long long trigger_counter, const rclcpp::Time & stamp,
    int64_t exposure_lines, const uint8_t * data, size_t size) override
  {
    if (!enabled() || data == nullptr || size == 0 || (frame_index % every_n_) != 0) {
      return;
    }
    const int64_t stamp_ns = stamp.nanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
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
    // Two 4K UYVY frames per camera keep memory bounded while allowing the
    // encoder to absorb short scheduling spikes without blocking capture.
    if (queue_.size() >= 2) {
      ++dropped_queue_frames_;
      return;
    }
    Pending pending;
    pending.frame_index = frame_index;
    pending.trigger_counter = trigger_counter;
    pending.stamp_ns = stamp_ns;
    pending.exposure_lines = exposure_lines;
    pending.data.resize(size);
    std::memcpy(pending.data.data(), data, size);
    queue_.emplace_back(std::move(pending));
    ++written_;
    condition_.notify_one();
  }

  uint64_t dropped_queue_frames() const { return dropped_queue_frames_.load(); }

  void close() override
  {
    if (closed_.exchange(true)) {
      return;
    }
    enabled_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      appsrc_ = nullptr;
      appsink_ = nullptr;
    }
    stream_.flush();
    index_.flush();
    stream_.close();
    index_.close();
  }

private:
  struct Pending
  {
    uint64_t frame_index {};
    long long trigger_counter {};
    int64_t stamp_ns {};
    int64_t exposure_lines {};
    std::vector<uint8_t> data;
  };

  void create_pipeline()
  {
    pipeline_ = gst_pipeline_new(nullptr);
    appsrc_ = gst_element_factory_make("appsrc", nullptr);
    GstElement * encoder = gst_element_factory_make("mpph264enc", nullptr);
    GstElement * parser = gst_element_factory_make("h264parse", nullptr);
    appsink_ = gst_element_factory_make("appsink", nullptr);
    if (pipeline_ == nullptr || appsrc_ == nullptr || encoder == nullptr ||
      parser == nullptr || appsink_ == nullptr)
    {
      throw std::runtime_error("create GStreamer MPP H.264 pipeline failed");
    }

    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "UYVY", "width", G_TYPE_INT, width_,
      "height", G_TYPE_INT, height_, "framerate", GST_TYPE_FRACTION, 10, 1, nullptr);
    g_object_set(
      appsrc_, "is-live", FALSE, "format", GST_FORMAT_TIME, "block", TRUE,
      "max-bytes", static_cast<guint64>(stride_) * static_cast<guint64>(height_) * 2, nullptr);
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);
    // Every frame is an IDR, and headers are repeated so each indexed AU can
    // be decoded independently after seeking by the CSV offset.
    g_object_set(encoder, "gop", 1, "header-mode", 1, "profile", 66, "max-pending", 4, nullptr);
    g_object_set(parser, "config-interval", -1, nullptr);
    g_object_set(appsink_, "sync", FALSE, "max-buffers", 2, "drop", FALSE, nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, encoder, parser, appsink_, nullptr);
    if (!gst_element_link_many(appsrc_, encoder, parser, appsink_, nullptr)) {
      throw std::runtime_error("link GStreamer MPP H.264 pipeline failed");
    }
    const auto state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("start GStreamer MPP H.264 pipeline failed");
    }
  }

  void worker_loop()
  {
    while (true) {
      Pending pending;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&]() { return stopping_ || !queue_.empty(); });
        if (queue_.empty() && stopping_) {
          break;
        }
        pending = std::move(queue_.front());
        queue_.pop_front();
      }

      GstBuffer * buffer = gst_buffer_new_allocate(nullptr, pending.data.size(), nullptr);
      if (buffer == nullptr) {
        fail_pipeline("allocate GStreamer input buffer failed");
        break;
      }
      GstMapInfo map {};
      if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        fail_pipeline("map GStreamer input buffer failed");
        break;
      }
      std::memcpy(map.data, pending.data.data(), pending.data.size());
      gst_buffer_unmap(buffer, &map);
      GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DURATION(buffer) = GST_SECOND / 10;
      const auto flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
      if (flow != GST_FLOW_OK) {
        fail_pipeline("push GStreamer input buffer failed");
        break;
      }
      GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
      if (sample == nullptr) {
        fail_pipeline("pull encoded H.264 sample failed");
        break;
      }
      GstBuffer * encoded = gst_sample_get_buffer(sample);
      GstMapInfo encoded_map {};
      if (encoded == nullptr || !gst_buffer_map(encoded, &encoded_map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        fail_pipeline("map encoded H.264 sample failed");
        break;
      }
      const auto position = stream_.tellp();
      if (position < 0) {
        gst_buffer_unmap(encoded, &encoded_map);
        gst_sample_unref(sample);
        fail_pipeline("query H.264 stream position failed");
        break;
      }
      const size_t encoded_size = encoded_map.size;
      stream_.write(reinterpret_cast<const char *>(encoded_map.data), encoded_map.size);
      gst_buffer_unmap(encoded, &encoded_map);
      gst_sample_unref(sample);
      if (!stream_) {
        fail_pipeline("write H.264 stream failed");
        break;
      }
      index_ << pending.frame_index << ',' << pending.trigger_counter << ',' << pending.stamp_ns << ','
             << pending.exposure_lines << ',' << static_cast<uint64_t>(position) << ','
             << encoded_size << ',' << width_ << ',' << height_ << ',' << stride_ << ",h264\n";
      if (!index_) {
        fail_pipeline("write H.264 index failed");
        break;
      }
      if ((++encoded_frames_ % 10) == 0) {
        stream_.flush();
        index_.flush();
      }
    }
    if (appsrc_ != nullptr) {
      gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
      while (GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_))) {
        gst_sample_unref(sample);
      }
    }
  }

  void fail_pipeline(const char * message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_ = message;
    stopping_ = true;
    enabled_.store(false);
  }

  inline static std::once_flag gst_once_;
  int every_n_ {1};
  int width_ {};
  int height_ {};
  int stride_ {};
  double rate_hz_ {};
  std::filesystem::path stream_path_;
  std::filesystem::path index_path_;
  std::ofstream stream_;
  std::ofstream index_;
  std::atomic<bool> enabled_ {false};
  std::atomic<bool> closed_ {false};
  std::atomic<uint64_t> dropped_queue_frames_ {0};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Pending> queue_;
  std::thread worker_;
  bool stopping_ {false};
  std::string error_;
  uint64_t written_ {0};
  uint64_t encoded_frames_ {0};
  int64_t last_seen_stamp_ns_ {-1};
  double rate_credit_ {0.0};
  GstElement * pipeline_ {nullptr};
  GstElement * appsrc_ {nullptr};
  GstElement * appsink_ {nullptr};
};
