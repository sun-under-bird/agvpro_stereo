#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "iostream"
#include "sensor_msgs/msg/camera_info.hpp" 
#include "yaml-cpp/yaml.h"   
using namespace std::chrono;

class StereoCameraNode:public rclcpp::Node{
    public:
    StereoCameraNode():Node("stereo_camera_node")
    {
        //parameter
        this->declare_parameter<int>("left_cam_idx",0);
        this->declare_parameter<int>("right_cam_idx",2);
        this->declare_parameter<int>("width",320);
        this->declare_parameter<int>("height",240);
        this->declare_parameter<std::string>("left_calib_path", "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/left2.yaml");  // 左内参路径
        this->declare_parameter<std::string>("right_calib_path", "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/right2.yaml"); 

        //get parameter
        int left_idx=this->get_parameter("left_cam_idx").as_int();
        int right_idx=this->get_parameter("right_cam_idx").as_int();
        width_=this->get_parameter("width").as_int();
        height_=this->get_parameter("height").as_int();
        std::string left_calib_path = this->get_parameter("left_calib_path").as_string();
        std::string right_calib_path = this->get_parameter("right_calib_path").as_string();

        //read image
        cap_left_.open(left_idx, cv::CAP_V4L2);
        cap_right_.open(right_idx, cv::CAP_V4L2);
        RCLCPP_INFO(this->get_logger(),"Opening cameras...");
        if(!cap_left_.isOpened()||!cap_right_.isOpened()){
            RCLCPP_FATAL(this->get_logger(),"Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        //width and height
        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH,width_);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH,width_);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);
        cap_left_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('Y','U','Y','V'));
        cap_right_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('Y','U','Y','V'));

        // 加载相机信息和校正所需的矩阵
        left_camera_info_ = load_camera_info(left_calib_path, width_, height_);
        right_camera_info_ = load_camera_info(right_calib_path, width_, height_);
        // 初始化校正矩阵（提前计算，避免每次回调重复计算）
        init_undistort_matrices();

        if(left_camera_info_.width == 0 || right_camera_info_.width == 0){
            RCLCPP_FATAL(this->get_logger(), "Load camera calibration file failed!");
            rclcpp::shutdown();
            return;
        }

        // Publisher - 原始图像
        pub_left_raw_=this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw",10);
        pub_right_raw_=this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw",10);
        // Publisher - 校正后图像（新增）
        pub_left_rect_=this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_rect",10);
        pub_right_rect_=this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_rect",10);
        // Publisher - 相机信息
        pub_left_info_=this->create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info",10); 
        pub_right_info_=this->create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info",10); 

        // 定时器回调（≈30fps，33ms一次）
        timer_=this->create_wall_timer(milliseconds(33),std::bind(&StereoCameraNode::stereo_callback,this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started successfully!");
        RCLCPP_INFO(this->get_logger(), "Left camera: /dev/video%d, Right camera: /dev/video%d", left_idx, right_idx);
    }

    private:
    // 加载相机标定信息，并提取校正所需的矩阵
    sensor_msgs::msg::CameraInfo load_camera_info(const std::string& yaml_path, int width, int height) {
        sensor_msgs::msg::CameraInfo info_msg;
        try {
            YAML::Node calib_node = YAML::LoadFile(yaml_path);
            
            // 1. 基础信息
            info_msg.width = width;
            info_msg.height = height;
            info_msg.distortion_model = calib_node["distortion_model"].as<std::string>();

            // 2. 畸变系数 D (k1, k2, p1, p2, k3)
            std::vector<double> D_vec = calib_node["distortion_coefficients"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < D_vec.size() && i < 5; ++i) {
                info_msg.d.push_back(D_vec[i]);
            }
            while (info_msg.d.size() < 5) {
                info_msg.d.push_back(0.0);
            }

            // 3. 内参矩阵 K (3x3)
            std::vector<double> K_vec = calib_node["camera_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) {
                info_msg.k[i] = (i < K_vec.size()) ? K_vec[i] : 0.0;
            }

            // 4. 校正矩阵 R (3x3)
            std::vector<double> R_vec = calib_node["rectification_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 9; ++i) {
                info_msg.r[i] = (i < R_vec.size()) ? R_vec[i] : ((i % 4 == 0) ? 1.0 : 0.0);
            }

            // 5. 投影矩阵 P (3x4)
            std::vector<double> P_vec = calib_node["projection_matrix"]["data"].as<std::vector<double>>();
            for (size_t i = 0; i < 12; ++i) {
                info_msg.p[i] = (i < P_vec.size()) ? P_vec[i] : 0.0;
            }

            // 强制启用校正（适配RTAB-Map）
            info_msg.binning_x = 0;
            info_msg.binning_y = 0;
            info_msg.roi.do_rectify = true;
            info_msg.roi.x_offset = 0;
            info_msg.roi.y_offset = 0;
            info_msg.roi.width = width;
            info_msg.roi.height = height;

            RCLCPP_DEBUG(this->get_logger(), "Load calibration file success: %s", yaml_path.c_str());
        }
        catch(YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Parse YAML error: %s", e.what());
            // 无标定文件时生成默认内参（避免程序崩溃）
            info_msg.width = width;
            info_msg.height = height;
            info_msg.distortion_model = "plumb_bob";
            info_msg.k[0] = width; info_msg.k[4] = height; info_msg.k[8] = 1.0;
            info_msg.r[0] = 1.0; info_msg.r[4] = 1.0; info_msg.r[8] = 1.0;
            info_msg.p[0] = width; info_msg.p[5] = height; info_msg.p[10] = 1.0;
            info_msg.roi.do_rectify = true;
        }
        catch(std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading camera info: %s", e.what());
            info_msg.width = width;
            info_msg.height = height;
            info_msg.roi.do_rectify = true;
        }
        return info_msg;
    }

    // 初始化畸变校正矩阵（提前计算，提升效率）
    void init_undistort_matrices() {
        // 左相机校正矩阵
        cv::Mat left_K = cv::Mat(3, 3, CV_64F, left_camera_info_.k.data()).clone();
        cv::Mat left_D = cv::Mat(1, 5, CV_64F, left_camera_info_.d.data()).clone();
        cv::Mat left_R = cv::Mat(3, 3, CV_64F, left_camera_info_.r.data()).clone();
        cv::Mat left_P = cv::Mat(3, 4, CV_64F, left_camera_info_.p.data()).clone();
        // 计算校正映射表（remap用，比直接undistort更快）
        cv::initUndistortRectifyMap(left_K, left_D, left_R, left_P.colRange(0,3), 
                                   cv::Size(width_, height_), CV_16SC2, left_map1_, left_map2_);

        // 右相机校正矩阵
        cv::Mat right_K = cv::Mat(3, 3, CV_64F, right_camera_info_.k.data()).clone();
        cv::Mat right_D = cv::Mat(1, 5, CV_64F, right_camera_info_.d.data()).clone();
        cv::Mat right_R = cv::Mat(3, 3, CV_64F, right_camera_info_.r.data()).clone();
        cv::Mat right_P = cv::Mat(3, 4, CV_64F, right_camera_info_.p.data()).clone();
        cv::initUndistortRectifyMap(right_K, right_D, right_R, right_P.colRange(0,3), 
                                   cv::Size(width_, height_), CV_16SC2, right_map1_, right_map2_);

        RCLCPP_INFO(this->get_logger(), "Undistort matrices initialized!");
    }
    
    void stereo_callback(){
        cv::Mat frame_left, frame_right;
        // 非阻塞读取（避免帧堆积）
        cap_left_.grab();
        cap_right_.grab();
        bool left_ret = cap_left_.retrieve(frame_left);
        bool right_ret = cap_right_.retrieve(frame_right);

        if (!left_ret || !right_ret || frame_left.empty() || frame_right.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame from camera! (left: %d, right: %d)", left_ret, right_ret);
            return;
        }

        // ========== 1. 图像畸变校正（核心新增逻辑） ==========
        cv::Mat frame_left_rect, frame_right_rect;
        // 使用预计算的映射表做校正，效率更高
        cv::remap(frame_left, frame_left_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(frame_right, frame_right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

        // ========== 2. 发布原始图像 ==========
        auto msg_left_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left).toImageMsg();
        auto msg_right_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right).toImageMsg();
        
        // ========== 3. 发布校正后图像 ==========
        auto msg_left_rect = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left_rect).toImageMsg();
        auto msg_right_rect = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right_rect).toImageMsg();

        // ========== 时间戳严格递增（防止回跳） ==========
        //static rclcpp::Time base_time = this->now(); 
        //static uint64_t frame_count = 0; 
        // rclcpp::Duration step = rclcpp::Duration::from_nanoseconds(33333333); // 30Hz对应33ms
        rclcpp::Time current_time = this->get_clock()->now(); 
        // frame_count++;

        // 设置时间戳和frame_id
        msg_left_raw->header.stamp = current_time;
        msg_right_raw->header.stamp = current_time;
        msg_left_rect->header.stamp = current_time;
        msg_right_rect->header.stamp = current_time;

        msg_left_raw->header.frame_id = "left_camera_optical";
        msg_right_raw->header.frame_id = "right_camera_optical";
        msg_left_rect->header.frame_id = "left_camera_optical";
        msg_right_rect->header.frame_id = "right_camera_optical";

        // 发布相机信息（同步时间戳）
        sensor_msgs::msg::CameraInfo left_info = left_camera_info_;
        sensor_msgs::msg::CameraInfo right_info = right_camera_info_;
        left_info.header = msg_left_raw->header;
        right_info.header = msg_right_raw->header;

        // ========== 4. 执行发布 ==========
        pub_left_raw_->publish(*msg_left_raw);
        pub_right_raw_->publish(*msg_right_raw);
        pub_left_rect_->publish(*msg_left_rect);  // 校正后图像
        pub_right_rect_->publish(*msg_right_rect); // 校正后图像
        pub_left_info_->publish(left_info);
        pub_right_info_->publish(right_info);
    }

    // 成员变量
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_, height_;

    // 校正映射表（提前计算，提升效率）
    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_;  // 校正后左图
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_; // 校正后右图
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_; 
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 相机信息
    sensor_msgs::msg::CameraInfo left_camera_info_; 
    sensor_msgs::msg::CameraInfo right_camera_info_; 
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    // 设置日志级别为INFO（方便调试）
    rclcpp::Logger logger = rclcpp::get_logger("stereo_camera_node");
    // rclcpp::set_logger_level(logger.get_name(), rclcpp::Logger::Level::Info);
    
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}