#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
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
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<ins_camera::StreamDelegate> delegate_;
    std::mutex camera_mutex_;
    bool streaming_ = false;

public:
    CameraWrapper(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

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
        // -- 1. declare and fetch ROS2 parameters for video resolution.
        node_->declare_parameter("video_resolution", "RES_1152_1152P30");
        node_->declare_parameter("lrv_video_resolution", "RES_1440_720P30");

        std::string video_resolution_str = "RES_1152_1152P30"; // Default resolution
        std::string lrv_video_resolution_str = "RES_1440_720P30"; // Default resolution

        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No available camera devices found.");
            return -1;
        }

        cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam->Open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera.");
            return -1;
        }
        RCLCPP_INFO(node_->get_logger(), "Camera opened successfully.");
        discovery.FreeDeviceDescriptors(list);

        // Keep the delegate alive for the entire CameraSDK streaming session.
        delegate_ = std::make_shared<TestStreamDelegate>(node_);
        cam->SetStreamDelegate(delegate_);

        auto start = time(NULL);

        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0; //no offset from UTC

        cam->SyncLocalTimeToCamera(utc_time,offset_time);

        video_resolution_str = node_->get_parameter("video_resolution").as_string();
        lrv_video_resolution_str = node_->get_parameter("lrv_video_resolution").as_string();

        ins_camera::LiveStreamParam param;
        param.video_resolution = StringToVideoResolution(video_resolution_str);
        param.lrv_video_reslution = StringToVideoResolution(lrv_video_resolution_str);
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        RCLCPP_INFO(node_->get_logger(), "Starting live streaming with updated video resolution: %s and LRV resolution: %s",
                    video_resolution_str.c_str(), lrv_video_resolution_str.c_str());
        if (!cam->StartLiveStreaming(param)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start live streaming.");
            return -1;
        }

        streaming_ = true;
        RCLCPP_INFO(node_->get_logger(), "Live streaming started.");
        return 0;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insta_publisher");

    CameraWrapper camera(node);
    if (camera.run_camera() != 0) {
        camera.stop_and_close();
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(node);
    // Do not rely on destructor timing: release the CameraSDK session before
    // the ROS context and publishers are torn down.
    camera.stop_and_close();
    rclcpp::shutdown();
    return 0;
}
