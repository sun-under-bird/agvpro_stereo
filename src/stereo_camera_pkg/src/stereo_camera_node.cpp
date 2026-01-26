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
        // parameters
        this->declare_parameter<int>("left_cam_idx", 0);
        this->declare_parameter<int>("right_cam_idx", 2);
        this->declare_parameter<int>("width", 320);
        this->declare_parameter<int>("height", 240);
        this->declare_parameter<std::string>("left_calib_path",
            "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/left2.yaml");
        this->declare_parameter<std::string>("right_calib_path",
            "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/right2.yaml");

        // get parameter
        int left_idx = this->get_parameter("left_cam_idx").as_int();
        int right_idx = this->get_parameter("right_cam_idx").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        std::string left_calib_path = this->get_parameter("left_calib_path").as_string();
        std::string right_calib_path = this->get_parameter("right_calib_path").as_string();

        // open cameras
        cap_left_.open(left_idx, cv::CAP_V4L2);
        cap_right_.open(right_idx, cv::CAP_V4L2);
        RCLCPP_INFO(this->get_logger(), "Opening cameras...");
        if (!cap_left_.isOpened() || !cap_right_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        // set width/height
        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);

        // Force V4L2 format. If you insist on YUYV, make sure we convert to BGR before publishing.
        cap_left_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_right_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));

        // Ask OpenCV to convert to BGR if possible (some drivers respect this, some don't)
        cap_left_.set(cv::CAP_PROP_CONVERT_RGB, 1);
        cap_right_.set(cv::CAP_PROP_CONVERT_RGB, 1);

        // load camera info
        left_camera_info_ = load_camera_info(left_calib_path, width_, height_);
        right_camera_info_ = load_camera_info(right_calib_path, width_, height_);

        if (left_camera_info_.width == 0 || right_camera_info_.width == 0) {
            RCLCPP_FATAL(this->get_logger(), "Load camera calibration file failed!");
            rclcpp::shutdown();
            return;
        }

        // init rectify maps
        init_undistort_matrices();
        if (left_map1_.empty() || right_map1_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rectify maps are empty. Will publish raw only.");
        }

        // publishers
        pub_left_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw", 10);
        pub_right_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw", 10);
        pub_left_rect_ = this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_rect", 10);
        pub_right_rect_ = this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_rect", 10);
        pub_left_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info", 10);
        pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info", 10);

        // timer (≈15fps)
        timer_ = this->create_wall_timer(milliseconds(66), std::bind(&StereoCameraNode::stereo_callback, this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started successfully!");
        RCLCPP_INFO(this->get_logger(), "Left camera: /dev/video%d, Right camera: /dev/video%d", left_idx, right_idx);
    }

private:
    // Load camera info from YAML
    sensor_msgs::msg::CameraInfo load_camera_info(const std::string &yaml_path, int width, int height) {
        sensor_msgs::msg::CameraInfo info_msg;
        try {
            YAML::Node calib_node = YAML::LoadFile(yaml_path);

            info_msg.width = width;
            info_msg.height = height;
            info_msg.distortion_model = calib_node["distortion_model"].as<std::string>();

            // D
            std::vector<double> D_vec = calib_node["distortion_coefficients"]["data"].as<std::vector<double>>();
            info_msg.d.clear();
            for (size_t i = 0; i < D_vec.size() && i < 5; ++i) {
                info_msg.d.push_back(D_vec[i]);
            }
            while (info_msg.d.size() < 5) {
                info_msg.d.push_back(0.0);
            }

            // K
            std::vector<double> K_vec = calib_node["camera_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) {
                info_msg.k[i] = (i < K_vec.size()) ? K_vec[i] : 0.0;
            }

            // R
            std::vector<double> R_vec = calib_node["rectification_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) {
                info_msg.r[i] = (i < R_vec.size()) ? R_vec[i] : ((i % 4 == 0) ? 1.0 : 0.0);
            }

            // P
            std::vector<double> P_vec = calib_node["projection_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 12; ++i) {
                info_msg.p[i] = (i < P_vec.size()) ? P_vec[i] : 0.0;
            }

            // rectify flags
            info_msg.binning_x = 0;
            info_msg.binning_y = 0;
            info_msg.roi.do_rectify = true;
            info_msg.roi.x_offset = 0;
            info_msg.roi.y_offset = 0;
            info_msg.roi.width = width;
            info_msg.roi.height = height;

            RCLCPP_INFO(this->get_logger(), "Load calibration file success: %s", yaml_path.c_str());
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Parse YAML error (%s): %s", yaml_path.c_str(), e.what());
            // fallback
            info_msg.width = width;
            info_msg.height = height;
            info_msg.distortion_model = "plumb_bob";
            info_msg.d = {0, 0, 0, 0, 0};
            info_msg.k = {double(width), 0, double(width) / 2.0,
                          0, double(height), double(height) / 2.0,
                          0, 0, 1};
            info_msg.r = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};
            info_msg.p = {double(width), 0, double(width) / 2.0, 0,
                          0, double(height), double(height) / 2.0, 0,
                          0, 0, 1, 0};
            info_msg.roi.do_rectify = true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading camera info: %s", e.what());
            info_msg.width = width;
            info_msg.height = height;
            info_msg.roi.do_rectify = true;
        }
        return info_msg;
    }

    void init_undistort_matrices() {
        try {
            // Left
            cv::Mat left_K(3, 3, CV_64F, left_camera_info_.k.data());
            cv::Mat left_D(1, 5, CV_64F, left_camera_info_.d.data());
            cv::Mat left_R(3, 3, CV_64F, left_camera_info_.r.data());
            cv::Mat left_P(3, 4, CV_64F, left_camera_info_.p.data());

            cv::initUndistortRectifyMap(left_K.clone(), left_D.clone(), left_R.clone(),
                                        left_P.colRange(0, 3).clone(),
                                        cv::Size(width_, height_), CV_16SC2,
                                        left_map1_, left_map2_);

            // Right
            cv::Mat right_K(3, 3, CV_64F, right_camera_info_.k.data());
            cv::Mat right_D(1, 5, CV_64F, right_camera_info_.d.data());
            cv::Mat right_R(3, 3, CV_64F, right_camera_info_.r.data());
            cv::Mat right_P(3, 4, CV_64F, right_camera_info_.p.data());

            cv::initUndistortRectifyMap(right_K.clone(), right_D.clone(), right_R.clone(),
                                        right_P.colRange(0, 3).clone(),
                                        cv::Size(width_, height_), CV_16SC2,
                                        right_map1_, right_map2_);

            RCLCPP_INFO(this->get_logger(), "Undistort/rectify maps initialized!");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "init_undistort_matrices failed: %s", e.what());
            left_map1_.release(); left_map2_.release();
            right_map1_.release(); right_map2_.release();
        }
    }

    // Convert camera frame to BGR if needed (YUYV/YUY2 -> BGR)
    static cv::Mat ensure_bgr(const cv::Mat &in) {
        if (in.empty()) return in;

        // Already BGR
        if (in.type() == CV_8UC3) {
            return in;
        }

        // Typical YUYV from V4L2 is CV_8UC2
        if (in.type() == CV_8UC2) {
            cv::Mat out;
            cv::cvtColor(in, out, cv::COLOR_YUV2BGR_YUY2);
            return out;
        }

        // Grayscale -> BGR
        if (in.type() == CV_8UC1) {
            cv::Mat out;
            cv::cvtColor(in, out, cv::COLOR_GRAY2BGR);
            return out;
        }

        // Fallback: try convert
        cv::Mat out;
        try {
            in.convertTo(out, CV_8UC3);
        } catch (...) {
            return in;
        }
        return out;
    }

    void stereo_callback() {
        cv::Mat frame_left, frame_right;

        // Non-blocking grab/retrieve
        cap_left_.grab();
        cap_right_.grab();
        bool left_ret = cap_left_.retrieve(frame_left);
        bool right_ret = cap_right_.retrieve(frame_right);

        if (!left_ret || !right_ret || frame_left.empty() || frame_right.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame! (left: %d, right: %d)", left_ret, right_ret);
            return;
        }

        // Ensure BGR (IMPORTANT if device outputs YUYV)
        frame_left = ensure_bgr(frame_left);
        frame_right = ensure_bgr(frame_right);

        // Rectify (if maps exist)
        cv::Mat frame_left_rect, frame_right_rect;
        if (!left_map1_.empty() && !right_map1_.empty()) {
            cv::remap(frame_left, frame_left_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
            cv::remap(frame_right, frame_right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);
        } else {
            frame_left_rect = frame_left;
            frame_right_rect = frame_right;
        }

        // Timestamp
        rclcpp::Time current_time = this->get_clock()->now();

        // Build headers
        std_msgs::msg::Header hdr_left;
        hdr_left.stamp = current_time;
        hdr_left.frame_id = "left_camera_optical";

        std_msgs::msg::Header hdr_right;
        hdr_right.stamp = current_time;
        hdr_right.frame_id = "right_camera_optical";

        // Publish images
        auto msg_left_raw = cv_bridge::CvImage(hdr_left, "bgr8", frame_left).toImageMsg();
        auto msg_right_raw = cv_bridge::CvImage(hdr_right, "bgr8", frame_right).toImageMsg();
        auto msg_left_rect = cv_bridge::CvImage(hdr_left, "bgr8", frame_left_rect).toImageMsg();
        auto msg_right_rect = cv_bridge::CvImage(hdr_right, "bgr8", frame_right_rect).toImageMsg();

        pub_left_raw_->publish(*msg_left_raw);
        pub_right_raw_->publish(*msg_right_raw);
        pub_left_rect_->publish(*msg_left_rect);
        pub_right_rect_->publish(*msg_right_rect);

        // Publish camera info (IMPORTANT: align header with rect image)
        sensor_msgs::msg::CameraInfo left_info = left_camera_info_;
        sensor_msgs::msg::CameraInfo right_info = right_camera_info_;
        left_info.header = hdr_left;
        right_info.header = hdr_right;

        pub_left_info_->publish(left_info);
        pub_right_info_->publish(right_info);
    }

private:
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_{320}, height_{240};

    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;

    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
