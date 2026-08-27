#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;
using sensor_msgs::msg::PointField;

namespace
{

struct RenderPoint
{
  float forward;
  float lateral;
  float vertical;
  float intensity;
};

uint64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

void atomic_write(const fs::path & path, const std::vector<uint8_t> & data)
{
  const fs::path temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("open failed: " + temporary.string());
    }
    output.write(reinterpret_cast<const char *>(data.data()), data.size());
    if (!output) {
      throw std::runtime_error("write failed: " + temporary.string());
    }
  }
  fs::rename(temporary, path);
}

void atomic_write(const fs::path & path, const std::string & data)
{
  atomic_write(path, std::vector<uint8_t>(data.begin(), data.end()));
}

const PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & message, const std::string & name)
{
  for (const auto & field : message.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

float read_float(const uint8_t * point, const PointField & field)
{
  if (field.datatype == PointField::FLOAT32) {
    float value;
    std::memcpy(&value, point + field.offset, sizeof(value));
    return value;
  }
  if (field.datatype == PointField::FLOAT64) {
    double value;
    std::memcpy(&value, point + field.offset, sizeof(value));
    return static_cast<float>(value);
  }
  throw std::runtime_error("unsupported floating-point field type");
}

uint32_t read_unsigned(const uint8_t * point, const PointField & field)
{
  switch (field.datatype) {
    case PointField::UINT8:
      return *(point + field.offset);
    case PointField::UINT16: {
      uint16_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return value;
    }
    case PointField::UINT32: {
      uint32_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return value;
    }
    default:
      throw std::runtime_error("unsupported unsigned field type");
  }
}

std::string stamps_json(const std::deque<uint64_t> & stamps)
{
  std::ostringstream output;
  output << '[';
  bool first = true;
  for (const auto stamp : stamps) {
    if (!first) {
      output << ',';
    }
    first = false;
    output << stamp;
  }
  output << ']';
  return output.str();
}

class FastCalib2Monitor : public rclcpp::Node
{
public:
  FastCalib2Monitor()
  : Node("fast_calib2_monitor")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/imx586/cam1/image_preview");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/vanjee_points722");
    output_dir_ = declare_parameter<std::string>(
      "output_dir", "/userdata/fast_calib2_runtime/ui_workers");
    lidar_period_ms_ = declare_parameter<int>("lidar_period_ms", 500);
    if (lidar_period_ms_ <= 0) {
      throw std::runtime_error("lidar_period_ms must be positive");
    }
    fs::create_directories(output_dir_);

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&FastCalib2Monitor::on_image, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, qos,
      std::bind(&FastCalib2Monitor::on_lidar, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "image=%s lidar=%s output=%s lidar_period_ms=%d",
      image_topic_.c_str(), lidar_topic_.c_str(), output_dir_.c_str(), lidar_period_ms_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      if (message->encoding != "mono8" || message->width == 0 || message->height == 0) {
        throw std::runtime_error("expected a non-empty mono8 preview");
      }
      if (message->data.size() < static_cast<size_t>(message->step) * message->height) {
        throw std::runtime_error("short preview image");
      }
      cv::Mat gray(
        static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC1,
        const_cast<uint8_t *>(message->data.data()), message->step);
      cv::Mat annotated;
      cv::cvtColor(gray, annotated, cv::COLOR_GRAY2BGR);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(
        gray, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), corners, ids);
      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(annotated, corners, ids);
      }
      std::sort(ids.begin(), ids.end());
      ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
      const bool ready =
        std::find(ids.begin(), ids.end(), 1) != ids.end() &&
        std::find(ids.begin(), ids.end(), 2) != ids.end() &&
        std::find(ids.begin(), ids.end(), 3) != ids.end() &&
        std::find(ids.begin(), ids.end(), 4) != ids.end();

      std::vector<uint8_t> encoded;
      cv::imencode(".jpg", annotated, encoded, {cv::IMWRITE_JPEG_QUALITY, 82});
      atomic_write(fs::path(output_dir_) / "camera.jpg", encoded);
      image_stamps_.push_back(stamp_ns(message->header.stamp));
      while (image_stamps_.size() > 12) {
        image_stamps_.pop_front();
      }

      std::ostringstream json;
      json << "{\"frame_id\":\"" << message->header.frame_id
           << "\",\"markers\":[";
      for (size_t i = 0; i < ids.size(); ++i) {
        if (i) {
          json << ',';
        }
        json << ids[i];
      }
      json << "],\"markers_ready\":" << (ready ? "true" : "false")
           << ",\"preview_encoding\":\"mono8\""
           << ",\"preview_height\":" << message->height
           << ",\"preview_width\":" << message->width
           << ",\"stamps_ns\":" << stamps_json(image_stamps_)
           << ",\"updated_monotonic\":" << monotonic_seconds() << "}\n";
      atomic_write(fs::path(output_dir_) / "image.json", json.str());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "image: %s", error.what());
    }
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_lidar_ != std::chrono::steady_clock::time_point() &&
      now - last_lidar_ < std::chrono::milliseconds(lidar_period_ms_))
    {
      return;
    }
    last_lidar_ = now;
    try {
      if (message->is_bigendian) {
        throw std::runtime_error("big-endian point clouds are unsupported");
      }
      const auto * x_field = find_field(*message, "x");
      const auto * y_field = find_field(*message, "y");
      const auto * z_field = find_field(*message, "z");
      const auto * intensity_field = find_field(*message, "intensity");
      if (!intensity_field) {
        intensity_field = find_field(*message, "reflectivity");
      }
      const auto * ring_field = find_field(*message, "ring");
      if (!x_field || !y_field || !z_field || !ring_field) {
        throw std::runtime_error("point cloud is missing x/y/z/ring");
      }

      const size_t total = static_cast<size_t>(message->width) * message->height;
      size_t finite = 0;
      float z_min = std::numeric_limits<float>::infinity();
      float z_max = -std::numeric_limits<float>::infinity();
      std::unordered_set<uint32_t> rings;
      std::vector<RenderPoint> render_points;
      render_points.reserve(12000);
      const size_t sample_stride = std::max<size_t>(1, total / 12000);
      size_t finite_index = 0;

      for (uint32_t row = 0; row < message->height; ++row) {
        const uint8_t * row_data = message->data.data() + static_cast<size_t>(row) * message->row_step;
        for (uint32_t column = 0; column < message->width; ++column) {
          const uint8_t * point = row_data + static_cast<size_t>(column) * message->point_step;
          const float x = read_float(point, *x_field);
          const float y = read_float(point, *y_field);
          const float z = read_float(point, *z_field);
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }
          ++finite;
          z_min = std::min(z_min, z);
          z_max = std::max(z_max, z);
          rings.insert(read_unsigned(point, *ring_field));
          if ((finite_index++ % sample_stride) == 0) {
            const float intensity = intensity_field ? read_float(point, *intensity_field) : 0.0F;
            render_points.push_back({-x, y, z, intensity});
          }
        }
      }
      const float z_span = finite ? z_max - z_min : 0.0F;
      const bool ready = total == 38400 && finite >= 10000 && rings.size() == 32 && z_span > 0.1F;
      atomic_write(fs::path(output_dir_) / "cloud.jpg", render_cloud(render_points));
      lidar_stamps_.push_back(stamp_ns(message->header.stamp));
      while (lidar_stamps_.size() > 12) {
        lidar_stamps_.pop_front();
      }

      std::ostringstream json;
      json << std::fixed << std::setprecision(6)
           << "{\"finite_points\":" << finite
           << ",\"frame_ready\":" << (ready ? "true" : "false")
           << ",\"ring_count\":" << rings.size()
           << ",\"stamps_ns\":" << stamps_json(lidar_stamps_)
           << ",\"total_points\":" << total
           << ",\"updated_monotonic\":" << monotonic_seconds()
           << ",\"z_span_m\":" << z_span << "}\n";
      atomic_write(fs::path(output_dir_) / "lidar.json", json.str());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "lidar: %s", error.what());
    }
  }

  static double monotonic_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static std::vector<uint8_t> render_cloud(const std::vector<RenderPoint> & points)
  {
    constexpr int width = 960;
    constexpr int height = 420;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(246, 247, 249));
    cv::line(canvas, {width / 2, 0}, {width / 2, height}, {205, 210, 216}, 1);
    std::vector<float> intensities;
    intensities.reserve(points.size());
    for (const auto & point : points) {
      intensities.push_back(point.intensity);
    }
    float threshold = std::numeric_limits<float>::infinity();
    if (!intensities.empty()) {
      const size_t index = intensities.size() * 92 / 100;
      std::nth_element(intensities.begin(), intensities.begin() + index, intensities.end());
      threshold = intensities[index];
    }

    auto plot = [&](bool front) {
      const int box_start = front ? width / 2 : 0;
      const float x_min = -12.0F;
      const float x_max = 12.0F;
      const float y_min = front ? -4.0F : 0.0F;
      const float y_max = front ? 4.0F : 25.0F;
      cv::rectangle(canvas, {box_start + 12, 30}, {box_start + width / 2 - 12, height - 25},
        {220, 224, 229}, 1);
      for (const auto & point : points) {
        const float x = point.lateral;
        const float y = front ? point.vertical : point.forward;
        if (x < x_min || x > x_max || y < y_min || y > y_max) {
          continue;
        }
        const int px = box_start + 13 + static_cast<int>(
          (x - x_min) / (x_max - x_min) * (width / 2 - 26));
        const int py = height - 26 - static_cast<int>(
          (y - y_min) / (y_max - y_min) * (height - 56));
        canvas.at<cv::Vec3b>(std::clamp(py, 0, height - 1), std::clamp(px, 0, width - 1)) =
          point.intensity >= threshold ? cv::Vec3b(32, 104, 212) : cv::Vec3b(116, 124, 134);
      }
    };
    plot(false);
    plot(true);
    cv::putText(canvas, "TOP  lateral / forward", {16, 21}, cv::FONT_HERSHEY_SIMPLEX,
      0.52, {39, 43, 48}, 1, cv::LINE_AA);
    cv::putText(canvas, "FRONT  lateral / height", {width / 2 + 16, 21},
      cv::FONT_HERSHEY_SIMPLEX, 0.52, {39, 43, 48}, 1, cv::LINE_AA);
    std::vector<uint8_t> encoded;
    cv::imencode(".jpg", canvas, encoded, {cv::IMWRITE_JPEG_QUALITY, 84});
    return encoded;
  }

  std::string image_topic_;
  std::string lidar_topic_;
  std::string output_dir_;
  int lidar_period_ms_ {};
  std::chrono::steady_clock::time_point last_lidar_ {};
  std::deque<uint64_t> image_stamps_;
  std::deque<uint64_t> lidar_stamps_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FastCalib2Monitor>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "fast_calib2_monitor_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
