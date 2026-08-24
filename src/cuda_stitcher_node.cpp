#include "insta360_ros2_equilib_driver/cuda_stitcher.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace insta360_ros2_equilib_driver
{

class CudaStitcherNode final : public rclcpp::Node
{
public:
  CudaStitcherNode()
  : Node("cuda_stitcher")
  {
    declare_parameter<double>("cx_offset", 0.0);
    declare_parameter<double>("cy_offset", 0.0);
    declare_parameter<int>("crop_size", 1152);
    declare_parameter<std::vector<double>>("translation", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("rotation_deg", {0.0, 0.0, 0.0});
    declare_parameter<int>("out_width", 2304);
    declare_parameter<int>("out_height", 1152);
    declare_parameter<int>("source_width", 2304);
    declare_parameter<int>("source_height", 1152);
    declare_parameter<double>("fisheye_fov_deg", 195.0);
    declare_parameter<int>("cuda_device", 0);
    declare_parameter<bool>("publish_equirectangular", true);

    output_width_ = get_parameter("out_width").as_int();
    output_height_ = get_parameter("out_height").as_int();
    publish_enabled_ = get_parameter("publish_equirectangular").as_bool();

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::Image>("/equirectangular/image", qos);
    const auto configured_source_width = get_parameter("source_width").as_int();
    const auto configured_source_height = get_parameter("source_height").as_int();
    initialize_stitcher(configured_source_width, configured_source_height);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/dual_fisheye/image",
      qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          if (latest_frame_ != nullptr) {
            ++frames_dropped_;
          }
          latest_frame_ = std::move(message);
          ++frames_received_;
        }
        frame_condition_.notify_one();
      });

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&CudaStitcherNode::log_stats, this));
    worker_ = std::thread(&CudaStitcherNode::worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Native CUDA stitcher ready: output=%dx%d topic=/equirectangular/image",
      output_width_,
      output_height_);
  }

  ~CudaStitcherNode() override
  {
    stop_.store(true);
    frame_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    destroy_cuda_stitcher(stitcher_);
    stitcher_ = nullptr;
  }

private:
  bool initialize_stitcher(std::uint32_t source_width, std::uint32_t source_height)
  {
    const auto translation = get_parameter("translation").as_double_array();
    const auto rotation = get_parameter("rotation_deg").as_double_array();
    if (translation.size() != 3U || rotation.size() != 3U) {
      RCLCPP_ERROR(get_logger(), "translation and rotation_deg must contain three values");
      return false;
    }

    StitchParameters parameters{};
    parameters.source_width = static_cast<int>(source_width);
    parameters.source_height = static_cast<int>(source_height);
    parameters.output_width = output_width_;
    parameters.output_height = output_height_;
    parameters.crop_size = get_parameter("crop_size").as_int();
    parameters.cx_offset = static_cast<float>(get_parameter("cx_offset").as_double());
    parameters.cy_offset = static_cast<float>(get_parameter("cy_offset").as_double());
    parameters.translation_x = static_cast<float>(translation[0]);
    parameters.translation_y = static_cast<float>(translation[1]);
    parameters.translation_z = static_cast<float>(translation[2]);
    parameters.rotation_roll_deg = static_cast<float>(rotation[0]);
    parameters.rotation_pitch_deg = static_cast<float>(rotation[1]);
    parameters.rotation_yaw_deg = static_cast<float>(rotation[2]);
    parameters.fisheye_fov_deg =
      static_cast<float>(get_parameter("fisheye_fov_deg").as_double());
    parameters.cuda_device = get_parameter("cuda_device").as_int();

    std::string error;
    stitcher_ = create_cuda_stitcher(parameters, error);
    if (stitcher_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "CUDA stitcher initialization failed: %s", error.c_str());
      return false;
    }

    source_width_ = source_width;
    source_height_ = source_height;
    RCLCPP_INFO(
      get_logger(),
      "CUDA map initialized: %ux%u -> %dx%d on device %d",
      source_width,
      source_height,
      output_width_,
      output_height_,
      parameters.cuda_device);
    return true;
  }

  void worker_loop()
  {
    while (!stop_.load()) {
      sensor_msgs::msg::Image::ConstSharedPtr source;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_condition_.wait(
          lock, [this]() {return stop_.load() || latest_frame_ != nullptr;});
        if (stop_.load()) {
          return;
        }
        source = std::move(latest_frame_);
        latest_frame_.reset();
      }

      if (source->encoding != sensor_msgs::image_encodings::BGR8) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Expected bgr8 input, received '%s'", source->encoding.c_str());
        continue;
      }
      if (!publish_enabled_) {
        continue;
      }
      if (
        stitcher_ == nullptr ||
        source->width != source_width_ ||
        source->height != source_height_)
      {
        destroy_cuda_stitcher(stitcher_);
        stitcher_ = nullptr;
        if (!initialize_stitcher(source->width, source->height)) {
          continue;
        }
      }

      float cuda_ms = 0.0F;
      std::string error;
      if (!run_cuda_stitcher(
          stitcher_, source->data.data(), source->step, cuda_ms, error))
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CUDA stitching failed: %s", error.c_str());
        continue;
      }

      auto output = std::make_unique<sensor_msgs::msg::Image>();
      output->header = source->header;
      output->height = static_cast<std::uint32_t>(output_height_);
      output->width = static_cast<std::uint32_t>(output_width_);
      output->encoding = sensor_msgs::image_encodings::BGR8;
      output->is_bigendian = false;
      output->step = static_cast<std::uint32_t>(output_width_ * 3);
      const auto * output_data = cuda_stitcher_output(stitcher_);
      const auto output_size = cuda_stitcher_output_size(stitcher_);
      output->data.resize(output_size);
      std::memcpy(output->data.data(), output_data, output_size);

      publisher_->publish(std::move(output));
      ++frames_published_;
      cuda_time_us_.fetch_add(
        static_cast<std::uint64_t>(cuda_ms * 1000.0F), std::memory_order_relaxed);
      auto previous_max = max_cuda_time_us_.load(std::memory_order_relaxed);
      const auto current_us = static_cast<std::uint64_t>(cuda_ms * 1000.0F);
      while (
        current_us > previous_max &&
        !max_cuda_time_us_.compare_exchange_weak(previous_max, current_us))
      {
      }
    }
  }

  void log_stats()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_stats_time_).count();
    const auto received = frames_received_.load();
    const auto published = frames_published_.load();
    const auto dropped = frames_dropped_.load();
    const auto received_delta = received - last_received_;
    const auto published_delta = published - last_published_;
    const auto dropped_delta = dropped - last_dropped_;
    const auto cuda_time = cuda_time_us_.exchange(0);
    const auto max_cuda_time = max_cuda_time_us_.exchange(0);
    const double average_cuda_ms =
      published_delta == 0U ? 0.0 :
      static_cast<double>(cuda_time) / static_cast<double>(published_delta) / 1000.0;

    RCLCPP_INFO(
      get_logger(),
      "[STATS] input=%.1f fps equirect=%.1f fps dropped=%lu "
      "cuda=%.2f ms max_cuda=%.2f ms subscribers=%zu wall_ns=%lld",
      static_cast<double>(received_delta) / elapsed,
      static_cast<double>(published_delta) / elapsed,
      static_cast<unsigned long>(dropped_delta),
      average_cuda_ms,
      static_cast<double>(max_cuda_time) / 1000.0,
      publisher_->get_subscription_count(),
      static_cast<long long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count()));

    last_stats_time_ = now;
    last_received_ = received;
    last_published_ = published;
    last_dropped_ = dropped;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_frame_;
  std::thread worker_;
  std::atomic<bool> stop_{false};

  CudaStitcher * stitcher_{nullptr};
  std::uint32_t source_width_{0};
  std::uint32_t source_height_{0};
  int output_width_{2304};
  int output_height_{1152};
  bool publish_enabled_{true};

  std::atomic<std::uint64_t> frames_received_{0};
  std::atomic<std::uint64_t> frames_published_{0};
  std::atomic<std::uint64_t> frames_dropped_{0};
  std::atomic<std::uint64_t> cuda_time_us_{0};
  std::atomic<std::uint64_t> max_cuda_time_us_{0};
  std::chrono::steady_clock::time_point last_stats_time_{std::chrono::steady_clock::now()};
  std::uint64_t last_received_{0};
  std::uint64_t last_published_{0};
  std::uint64_t last_dropped_{0};
};

}  // namespace insta360_ros2_equilib_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<insta360_ros2_equilib_driver::CudaStitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
