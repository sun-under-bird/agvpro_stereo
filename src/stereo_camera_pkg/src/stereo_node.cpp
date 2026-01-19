#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "iostream"
// 移除不需要的头文件
// #include "sensor_msgs/msg/camera_info.hpp" 
// #include "yaml-cpp/yaml.h"   

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
        // 移除标定文件路径相关参数
        // this->declare_parameter<std::string>("left_calib_path", "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/left.yaml");
        // this->declare_parameter<std::string>("right_calib_path", "/home/yahboom/camera_ws/src/stereo_camera_pkg/config/right.yaml"); 

        //get parameter
        int left_idx=this->get_parameter("left_cam_idx").as_int();
        int right_idx=this->get_parameter("right_cam_idx").as_int();
        width_=this->get_parameter("width").as_int();
        height_=this->get_parameter("height").as_int();
        // 移除标定文件路径参数读取
        // std::string left_calib_path = this->get_parameter("left_calib_path").as_string();
        // std::string right_calib_path = this->get_parameter("right_calib_path").as_string();

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

        // 移除所有标定文件加载、校正矩阵初始化逻辑
        // left_camera_info_ = load_camera_info(left_calib_path, width_, height_);
        // right_camera_info_ = load_camera_info(right_calib_path, width_, height_);
        // init_undistort_matrices();

        // if(left_camera_info_.width == 0 || right_camera_info_.width == 0){
        //     RCLCPP_FATAL(this->get_logger(), "Load camera calibration file failed!");
        //     rclcpp::shutdown();
        //     return;
        // }

        // Publisher - 仅保留原始图像发布器
        pub_left_raw_=this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_raw",10);
        pub_right_raw_=this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw",10);
        // 移除校正后图像、相机信息发布器
        // pub_left_rect_=this->create_publisher<sensor_msgs::msg::Image>("/left_camera/image_rect",10);
        // pub_right_rect_=this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_rect",10);
        // pub_left_info_=this->create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info",10); 
        // pub_right_info_=this->create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info",10); 

        // 定时器回调（≈30fps，33ms一次）
        timer_=this->create_wall_timer(milliseconds(33),std::bind(&StereoCameraNode::stereo_callback,this));

        RCLCPP_INFO(this->get_logger(), "Stereo camera node started successfully!");
        RCLCPP_INFO(this->get_logger(), "Left camera: /dev/video%d, Right camera: /dev/video%d", left_idx, right_idx);
    }

    private:
    // 移除加载相机标定信息的函数
    // sensor_msgs::msg::CameraInfo load_camera_info(const std::string& yaml_path, int width, int height) { ... }

    // 移除校正矩阵初始化函数
    // void init_undistort_matrices() { ... }
    
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

        // 移除图像畸变校正逻辑
        // cv::Mat frame_left_rect, frame_right_rect;
        // cv::remap(frame_left, frame_left_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
        // cv::remap(frame_right, frame_right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

        // 仅发布原始图像
        auto msg_left_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left).toImageMsg();
        auto msg_right_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right).toImageMsg();
        
        // 移除校正后图像消息创建
        // auto msg_left_rect = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left_rect).toImageMsg();
        // auto msg_right_rect = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right_rect).toImageMsg();

        // 时间戳严格递增（防止回跳）
        static rclcpp::Time base_time = this->now(); 
        static uint64_t frame_count = 0; 
        rclcpp::Duration step = rclcpp::Duration::from_nanoseconds(33333333); // 30Hz对应33ms
        rclcpp::Time current_time = base_time + (step * frame_count);
        frame_count++;

        // 设置时间戳和frame_id
        msg_left_raw->header.stamp = current_time;
        msg_right_raw->header.stamp = current_time;
        // 移除校正后图像时间戳设置
        // msg_left_rect->header.stamp = current_time;
        // msg_right_rect->header.stamp = current_time;

        msg_left_raw->header.frame_id = "left_camera";
        msg_right_raw->header.frame_id = "right_camera";
        // 移除校正后图像frame_id设置
        // msg_left_rect->header.frame_id = "left_camera";
        // msg_right_rect->header.frame_id = "right_camera";

        // 移除相机信息话题相关逻辑
        // sensor_msgs::msg::CameraInfo left_info = left_camera_info_;
        // sensor_msgs::msg::CameraInfo right_info = right_camera_info_;
        // left_info.header = msg_left_raw->header;
        // right_info.header = msg_right_raw->header;

        // 仅发布原始图像
        pub_left_raw_->publish(*msg_left_raw);
        pub_right_raw_->publish(*msg_right_raw);
        // 移除校正后图像、相机信息发布
        // pub_left_rect_->publish(*msg_left_rect);
        // pub_right_rect_->publish(*msg_right_rect);
        // pub_left_info_->publish(left_info);
        // pub_right_info_->publish(right_info);
    }

    // 成员变量
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    int width_, height_;

    // 移除校正映射表相关变量
    // cv::Mat left_map1_, left_map2_;
    // cv::Mat right_map1_, right_map2_;

    // 仅保留原始图像发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_raw_;
    // 移除校正后图像、相机信息发布器
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_;
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_; 
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 移除相机信息相关变量
    // sensor_msgs::msg::CameraInfo left_camera_info_; 
    // sensor_msgs::msg::CameraInfo right_camera_info_; 
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