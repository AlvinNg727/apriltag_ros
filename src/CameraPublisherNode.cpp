// V4L2 camera publisher component node.
//
// Publishes a V4L2 camera as image_raw + camera_info via
// image_transport::CameraPublisher. Calibrate with:
//     ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
//         image:=/down_camera/image_raw camera:=/down_camera
// then save the result over apriltag_ros/cfg/camera/down_camera.yaml.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace
{

    // Opens a V4L2 camera and configures the capture format.
    class V4L2Capture {
    public:
        V4L2Capture(const std::string& device, int width, int height, double fps) : cap_(device, cv::CAP_V4L2)
        {
            if(!cap_.isOpened()) {
                throw std::runtime_error("Could not open camera " + device + " via V4L2");
            }

            // Set MJPEG format first
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            cap_.set(cv::CAP_PROP_FPS, fps);

            // Disable auto-exposure to prevent frame rate drops in low light
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);

            // Manual image controls for the new camera
            cap_.set(cv::CAP_PROP_BRIGHTNESS, 0);
            cap_.set(cv::CAP_PROP_SATURATION, 0);
            cap_.set(cv::CAP_PROP_EXPOSURE, 10000);
            cap_.set(cv::CAP_PROP_GAIN, 0);
        }

        V4L2Capture(const V4L2Capture&) = delete;
        V4L2Capture& operator=(const V4L2Capture&) = delete;

        bool read(cv::Mat& frame)
        {
            return cap_.read(frame) && !frame.empty();
        }

    private:
        cv::VideoCapture cap_;
    };

}// namespace

class CameraPublisherNode : public rclcpp::Node {
public:
    explicit CameraPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("camera_publisher", options)
    {
        const std::string video_device = declare_parameter<std::string>(
            "video_device", "/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0");
        const int width = declare_parameter("image_width", 1600);
        const int height = declare_parameter("image_height", 1200);
        const double framerate = declare_parameter("framerate", 60.0);
        frame_id_ = declare_parameter<std::string>("frame_id", "down_camera_optical_frame");

        if(framerate <= 0.0) {
            throw std::invalid_argument("Parameter 'framerate' must be positive");
        }

        const std::string camera_name = declare_parameter<std::string>("camera_name", "down_camera");
        const std::string camera_info_url = declare_parameter<std::string>(
            "camera_info_url", "package://apriltag_ros/cfg/camera/down_camera.yaml");

#ifdef image_transport_QoS
        const rclcpp::QoS qos = rclcpp::SensorDataQoS();
#else
        const rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
#endif
        pub_cam_ = image_transport::create_camera_publisher(
#ifdef image_transport_NODE_INTERFACE
            image_transport::RequiredInterfaces{*this},
#else
            this,
#endif
            "image_raw", qos);

        info_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name, camera_info_url);
        if(!info_mgr_->isCalibrated()) {
            RCLCPP_WARN(get_logger(), "Camera is uncalibrated, publishing null CameraInfo.");
        }

        cap_ = std::make_unique<V4L2Capture>(video_device, width, height, framerate);

        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / framerate));
        timer_ = create_wall_timer(period, [this]() { publishFrame(); });

        RCLCPP_INFO(get_logger(), "Publishing %s on %s/image_raw", video_device.c_str(), get_namespace());
    }

private:
    void publishFrame()
    {
        cv::Mat frame;
        if(!cap_->read(frame)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No frame from camera");
            return;
        }

        const auto stamp = get_clock()->now();

        cv::Mat gray;
        if(frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        }
        else {
            gray = frame;
        }

        cv_bridge::CvImage img;
        img.header.stamp = stamp;
        img.header.frame_id = frame_id_;
        img.encoding = sensor_msgs::image_encodings::MONO8;
        img.image = gray;

        auto info = info_mgr_->getCameraInfo();
        info.header.stamp = stamp;
        info.header.frame_id = frame_id_;

        pub_cam_.publish(*img.toImageMsg(), info);
    }

    std::string frame_id_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> info_mgr_;
    image_transport::CameraPublisher pub_cam_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Declared last so the device closes before the publishers are torn down.
    std::unique_ptr<V4L2Capture> cap_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraPublisherNode)
