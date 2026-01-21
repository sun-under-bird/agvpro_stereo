#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono;

class StereoCameraNode : public rclcpp::Node {
public:
    StereoCameraNode() : Node("stereo_camera_node") {
        this->declare_parameter<int>("left_cam_idx", 0);
        this->declare_parameter<int>("right_cam_idx", 2);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<std::string>("left_calib_path",
            "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/left2.yaml");
        this->declare_parameter<std::string>("right_calib_path",
            "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/right2.yaml");

        int left_idx = this->get_parameter("left_cam_idx").as_int();
        int right_idx = this->get_parameter("right_cam_idx").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        std::string left_calib_path = this->get_parameter("left_calib_path").as_string();
        std::string right_calib_path = this->get_parameter("right_calib_path").as_string();

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

        left_camera_info_  = load_camera_info(left_calib_path,  width_, height_);
        right_camera_info_ = load_camera_info(right_calib_path, width_, height_);

        if (left_camera_info_.width == 0 || right_camera_info_.width == 0) {
            RCLCPP_FATAL(this->get_logger(), "Load camera calibration file failed!");
            rclcpp::shutdown();
            return;
        }

        pub_left_raw_  = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw", 10);
        pub_right_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw", 10);
        pub_left_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info", 10);
        pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info", 10);

        timer_ = this->create_wall_timer(milliseconds(33), std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started!");
        RCLCPP_INFO(this->get_logger(), "Left /dev/video%d, Right /dev/video%d, size=%dx%d fps=%d",
                    left_idx, right_idx, width_, height_, fps_);

        RCLCPP_INFO(this->get_logger(), "Actual L=%.0fx%.0f fps=%.1f | R=%.0fx%.0f fps=%.1f",
                    cap_left_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_left_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_left_.get(cv::CAP_PROP_FPS),
                    cap_right_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_right_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_right_.get(cv::CAP_PROP_FPS));

        RCLCPP_INFO(this->get_logger(), "Right P[3]=%.6f (stereo should usually be != 0)", right_camera_info_.p[3]);
    }

private:
    static void fill_array_9(const std::vector<double>& v, std::array<double, 9>& out) {
        for (size_t i = 0; i < 9; ++i) out[i] = (i < v.size()) ? v[i] : 0.0;
    }
    static void fill_array_12(const std::vector<double>& v, std::array<double, 12>& out) {
        for (size_t i = 0; i < 12; ++i) out[i] = (i < v.size()) ? v[i] : 0.0;
    }

    sensor_msgs::msg::CameraInfo load_camera_info(const std::string& yaml_path, int width, int height) {
        sensor_msgs::msg::CameraInfo info;
        try {
            YAML::Node calib = YAML::LoadFile(yaml_path);

            info.width = width;
            info.height = height;

            if (calib["image_width"] && calib["image_height"]) {
                int yw = calib["image_width"].as<int>();
                int yh = calib["image_height"].as<int>();
                if (yw != width || yh != height) {
                    RCLCPP_WARN(this->get_logger(),
                                "YAML (%dx%d) != requested (%dx%d). Best practice: capture at YAML resolution.",
                                yw, yh, width, height);
                }
            }

            info.distortion_model = calib["distortion_model"].as<std::string>();

            std::vector<double> D = calib["distortion_coefficients"]["data"].as<std::vector<double>>();
            info.d.clear();
            for (size_t i = 0; i < D.size(); ++i) info.d.push_back(D[i]);
            while (info.d.size() < 5) info.d.push_back(0.0);

            std::vector<double> K = calib["camera_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) info.k[i] = (i < K.size()) ? K[i] : 0.0;

            if (calib["rectification_matrix"] && calib["rectification_matrix"]["data"]) {
                std::vector<double> R = calib["rectification_matrix"]["data"].as<std::vector<double>>();
                fill_array_9(R, info.r);
            } else {
                RCLCPP_WARN(this->get_logger(), "No rectification_matrix in %s, using Identity R.", yaml_path.c_str());
                info.r = {1,0,0, 0,1,0, 0,0,1};
            }

            if (calib["projection_matrix"] && calib["projection_matrix"]["data"]) {
                std::vector<double> P = calib["projection_matrix"]["data"].as<std::vector<double>>();
                fill_array_12(P, info.p);
            } else {
                RCLCPP_WARN(this->get_logger(), "No projection_matrix in %s, using default P from K.", yaml_path.c_str());
                for (auto &x : info.p) x = 0.0;
                info.p[0] = info.k[0]; info.p[2] = info.k[2];
                info.p[5] = info.k[4]; info.p[6] = info.k[5];
                info.p[10] = 1.0;
            }

            info.binning_x = 0;
            info.binning_y = 0;
            info.roi.x_offset = 0;
            info.roi.y_offset = 0;
            info.roi.width = width;
            info.roi.height = height;
            info.roi.do_rectify = true;

            RCLCPP_INFO(this->get_logger(), "Loaded camera_info from %s", yaml_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s: %s", yaml_path.c_str(), e.what());
            info.width = 0;
            info.height = 0;
        }
        return info;
    }

    static cv::Mat ensure_bgr(const cv::Mat& frame, rclcpp::Logger logger, const char* which) {
        if (frame.empty()) return {};

        if (frame.type() == CV_8UC3) {
            return frame;
        }
        if (frame.type() == CV_8UC2) {
            cv::Mat bgr;
            cv::cvtColor(frame, bgr, cv::COLOR_YUV2BGR_YUY2);
            return bgr;
        }

        RCLCPP_WARN(logger, "%s unexpected frame type=%d channels=%d", which, frame.type(), frame.channels());
        return {};
    }

    void stereo_callback() {
        cv::Mat left_frame, right_frame;

        cap_left_.grab();
        cap_right_.grab();

        bool left_ok = cap_left_.retrieve(left_frame);
        bool right_ok = cap_right_.retrieve(right_frame);

        if (!left_ok || !right_ok || left_frame.empty() || right_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame! (left:%d right:%d)", left_ok, right_ok);
            return;
        }

        cv::Mat left_bgr = ensure_bgr(left_frame, this->get_logger(), "Left");
        cv::Mat right_bgr = ensure_bgr(right_frame, this->get_logger(), "Right");
        if (left_bgr.empty() || right_bgr.empty()) return;

        auto msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_bgr).toImageMsg();
        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_bgr).toImageMsg();

        rclcpp::Time stamp = this->get_clock()->now();
        msg_left->header.stamp = stamp;
        msg_right->header.stamp = stamp;
        msg_left->header.frame_id = "left_camera";
        msg_right->header.frame_id = "right_camera";

        sensor_msgs::msg::CameraInfo left_info = left_camera_info_;
        sensor_msgs::msg::CameraInfo right_info = right_camera_info_;
        left_info.header = msg_left->header;
        right_info.header = msg_right->header;

        pub_left_raw_->publish(*msg_left);
        pub_right_raw_->publish(*msg_right);
        pub_left_info_->publish(left_info);
        pub_right_info_->publish(right_info);
    }

    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_{320};
    int height_{240};
    int fps_{30};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
