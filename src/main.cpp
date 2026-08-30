#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <limits>
#include <mutex>

#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::atomic<uint64_t> video_frames_{0};
    std::atomic<int64_t> last_video_ns_{0};

    static int64_t steady_now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

public:
    TestStreamDelegate(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        // Publisher for the compressed H.264 video stream
        compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/image/compressed",
            rclcpp::QoS(10)
        );

        // Publisher for IMU data (remains the same)
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        RCLCPP_INFO(node_->get_logger(), "Publisher for compressed images and IMU created.");
    }

    virtual ~TestStreamDelegate() {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override {
        // We only care about the main video stream (index 0)
        if (stream_index == 0 && size > 0 && compressed_pub_) {
            video_frames_.fetch_add(1, std::memory_order_relaxed);
            last_video_ns_.store(steady_now_ns(), std::memory_order_relaxed);
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

            // Set the header
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "camera_frame";

            // Set the format to H.264
            // The subscriber will need to know this to select the correct decoder.
            msg->format = "h264";

            // Copy the compressed video data directly into the message
            msg->data.assign(data, data + size);

            compressed_pub_->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = node_->get_clock()->now();
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;

            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;

            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0; // Neutral orientation
            msg->orientation_covariance[0] = -1.0; // No orientation data available

            for (int i = 0; i < 9; i++)
            {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {}

    uint64_t video_frame_count() const {
        return video_frames_.load(std::memory_order_relaxed);
    }

    double seconds_since_last_video() const {
        const int64_t last_ns = last_video_ns_.load(std::memory_order_relaxed);
        if (last_ns == 0) {
            return std::numeric_limits<double>::infinity();
        }
        return static_cast<double>(steady_now_ns() - last_ns) / 1.0e9;
    }
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<TestStreamDelegate> delegate_;
    std::mutex camera_mutex_;
    bool streaming_ = false;
    std::chrono::steady_clock::time_point stream_started_at_{};

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
        node_->declare_parameter("video_resolution", "RES_1152_1152P30");
        node_->declare_parameter("lrv_video_resolution", "RES_1440_720P30");
        node_->declare_parameter("camera_retry_seconds", 2.0);
        node_->declare_parameter("stream_timeout_seconds", 5.0);
    }

    ~CameraWrapper() {
        stop_and_close();
    }

    void stop_and_close() {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!cam) {
            return;
        }

        // The SDK must stop the preview stream before Close(). Otherwise the
        // camera can retain a live-stream session and time out on the next Open().
        if (streaming_) {
            if (!cam->StopLiveStreaming()) {
                RCLCPP_WARN(node_->get_logger(), "StopLiveStreaming() reported a failure during shutdown");
            }
            streaming_ = false;
        }

        cam->Close();
        cam.reset();
        delegate_.reset();
        RCLCPP_INFO(node_->get_logger(), "Camera stream stopped and camera closed");
    }

    ins_camera::VideoResolution StringToVideoResolution(const std::string& res_str) {
        if (res_str == "RES_3840_1920P30") return ins_camera::VideoResolution::RES_3840_1920P30;
        if (res_str == "RES_2880_2880P30") return ins_camera::VideoResolution::RES_2880_2880P30;
        if (res_str == "RES_2560_1280P30") return ins_camera::VideoResolution::RES_2560_1280P30;
        if (res_str == "RES_1152_1152P30") return ins_camera::VideoResolution::RES_1152_1152P30;
        if (res_str == "RES_1920_960P30")  return ins_camera::VideoResolution::RES_1920_960P30;
        if (res_str == "RES_1440_720P30")  return ins_camera::VideoResolution::RES_1440_720P30;

        // Default fallback
        return ins_camera::VideoResolution::RES_1152_1152P30;
    }

    int run_camera() {
        std::string video_resolution_str = "RES_1152_1152P30"; // Default resolution
        std::string lrv_video_resolution_str = "RES_1440_720P30"; // Default resolution

        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No available camera devices found; will retry.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            discovery.FreeDeviceDescriptors(list);
            cam.reset();
            RCLCPP_WARN(node_->get_logger(), "Failed to open camera; will retry.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");
        discovery.FreeDeviceDescriptors(list);

        // Keep the delegate alive for the entire CameraSDK streaming session.
        delegate_ = std::make_shared<TestStreamDelegate>(node_);
        std::shared_ptr<ins_camera::StreamDelegate> sdk_delegate = delegate_;
        cam->SetStreamDelegate(sdk_delegate);

        auto start = time(NULL);

        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0; //no offset from UTC

        cam->SyncLocalTimeToCamera(utc_time,offset_time);

        video_resolution_str = node_->get_parameter("video_resolution").as_string();
        lrv_video_resolution_str = node_->get_parameter("lrv_video_resolution").as_string();

        ins_camera::LiveStreamParam param;
        param.video_resolution = StringToVideoResolution(video_resolution_str);
        param.lrv_video_resulution = StringToVideoResolution(lrv_video_resolution_str);
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        RCLCPP_INFO(node_->get_logger(), "Starting live streaming with updated video resolution: %s and LRV resolution: %s",
                    video_resolution_str.c_str(), lrv_video_resolution_str.c_str());
        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to start live streaming; will retry.");
            return -1;
        }

        streaming_ = true;
        stream_started_at_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }

    bool stream_timed_out(double timeout_seconds) const {
        if (!streaming_ || !delegate_) {
            return false;
        }
        const double startup_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stream_started_at_).count();
        if (delegate_->video_frame_count() == 0) {
            return startup_age > timeout_seconds;
        }
        return delegate_->seconds_since_last_video() > timeout_seconds;
    }

    uint64_t video_frame_count() const {
        return delegate_ ? delegate_->video_frame_count() : 0;
    }

    double retry_seconds() const {
        return node_->get_parameter("camera_retry_seconds").as_double();
    }

    double stream_timeout_seconds() const {
        return node_->get_parameter("stream_timeout_seconds").as_double();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_publisher");

    CameraWrapper camera(node);
    while (rclcpp::ok()) {
        if (camera.run_camera() != 0) {
            camera.stop_and_close();
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(
                std::chrono::duration<double>(camera.retry_seconds()));
            continue;
        }

        uint64_t previous_frames = camera.video_frame_count();
        auto previous_log = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            const auto now = std::chrono::steady_clock::now();
            const double log_elapsed = std::chrono::duration<double>(now - previous_log).count();
            if (log_elapsed >= 2.0) {
                const uint64_t current_frames = camera.video_frame_count();
                RCLCPP_INFO(
                    node->get_logger(),
                    "[CAMERA STATS] compressed=%.1f fps total=%lu wall_ns=%lld",
                    static_cast<double>(current_frames - previous_frames) / log_elapsed,
                    static_cast<unsigned long>(current_frames),
                    static_cast<long long>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()
                        ).count()
                    ));
                previous_frames = current_frames;
                previous_log = now;
            }
            if (camera.stream_timed_out(camera.stream_timeout_seconds())) {
                RCLCPP_WARN(
                    node->get_logger(),
                    "No H.264 packets received for %.1f seconds; restarting camera stream.",
                    camera.stream_timeout_seconds());
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        camera.stop_and_close();
        if (rclcpp::ok()) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(camera.retry_seconds()));
        }
    }

    rclcpp::shutdown();
    return 0;
}
