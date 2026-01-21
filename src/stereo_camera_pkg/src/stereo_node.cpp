#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>

using namespace std::chrono;

class StereoCameraNode : public rclcpp::Node {
public:
    StereoCameraNode() : Node("stereo_camera_node") {
        this->declare_parameter<int>("left_cam_idx", 0);
        this->declare_parameter<int>("right_cam_idx", 2);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 30);

        int left_idx = this->get_parameter("left_cam_idx").as_int();
        int right_idx = this->get_parameter("right_cam_idx").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        cap_left_.open(left_idx, cv::CAP_V4L2);
        cap_right_.open(right_idx, cv::CAP_V4L2);

        RCLCPP_INFO(this->get_logger(), "Opening cameras...");
        if (!cap_left_.isOpened() || !cap_right_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);

        cap_left_.set(cv::CAP_PROP_FPS, fps_);
        cap_right_.set(cv::CAP_PROP_FPS, fps_);

        cap_left_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_right_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));

        cap_left_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap_right_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        pub_left_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw", 10);
        pub_right_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw", 10);

        timer_ = this->create_wall_timer(milliseconds(33), std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started!");
        RCLCPP_INFO(this->get_logger(), "Left camera: /dev/video%d, Right camera: /dev/video%d", left_idx, right_idx);

        RCLCPP_INFO(this->get_logger(), "Requested size=%dx%d fps=%d", width_, height_, fps_);
        RCLCPP_INFO(this->get_logger(),
                    "Actual L=%.0fx%.0f fps=%.1f | R=%.0fx%.0f fps=%.1f",
                    cap_left_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_left_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_left_.get(cv::CAP_PROP_FPS),
                    cap_right_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_right_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_right_.get(cv::CAP_PROP_FPS));

        RCLCPP_INFO(this->get_logger(), "Left FOURCC=%s | Right FOURCC=%s",
                    fourcc_to_string(cap_left_).c_str(),
                    fourcc_to_string(cap_right_).c_str());
    }

private:
    static std::string fourcc_to_string(cv::VideoCapture &cap) {
        int fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
        char fourcc_str[] = {
            char(fourcc & 0xFF),
            char((fourcc >> 8) & 0xFF),
            char((fourcc >> 16) & 0xFF),
            char((fourcc >> 24) & 0xFF),
            0
        };
        return std::string(fourcc_str);
    }

    void stereo_callback() {
        cv::Mat left_frame, right_frame;

        cap_left_.grab();
        cap_right_.grab();

        bool left_ret = cap_left_.retrieve(left_frame);
        bool right_ret = cap_right_.retrieve(right_frame);

        if (!left_ret || !right_ret || left_frame.empty() || right_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame from camera! (left: %d, right: %d)", left_ret, right_ret);
            return;
        }

        static bool printed = false;
        if (!printed) {
            RCLCPP_INFO(this->get_logger(),
                        "Left frame: type=%d channels=%d depth=%d (CV_8UC2=%d CV_8UC3=%d)",
                        left_frame.type(), left_frame.channels(), left_frame.depth(),
                        CV_8UC2, CV_8UC3);
            RCLCPP_INFO(this->get_logger(),
                        "Right frame: type=%d channels=%d depth=%d (CV_8UC2=%d CV_8UC3=%d)",
                        right_frame.type(), right_frame.channels(), right_frame.depth(),
                        CV_8UC2, CV_8UC3);
            printed = true;
        }

        cv::Mat left_bgr, right_bgr;

        if (left_frame.type() == CV_8UC2) {
            cv::cvtColor(left_frame, left_bgr, cv::COLOR_YUV2BGR_YUY2);
        } else if (left_frame.type() == CV_8UC3) {
            left_bgr = left_frame;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected left frame type: %d", left_frame.type());
            return;
        }

        if (right_frame.type() == CV_8UC2) {
            cv::cvtColor(right_frame, right_bgr, cv::COLOR_YUV2BGR_YUY2);
        } else if (right_frame.type() == CV_8UC3) {
            right_bgr = right_frame;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected right frame type: %d", right_frame.type());
            return;
        }

        auto msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_bgr).toImageMsg();
        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_bgr).toImageMsg();

        rclcpp::Time current_time = this->get_clock()->now();
        msg_left->header.stamp = current_time;
        msg_right->header.stamp = current_time;

        msg_left->header.frame_id = "left_camera";
        msg_right->header.frame_id = "right_camera";

        pub_left_raw_->publish(*msg_left);
        pub_right_raw_->publish(*msg_right);
    }

    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_{640};
    int height_{480};
    int fps_{30};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
