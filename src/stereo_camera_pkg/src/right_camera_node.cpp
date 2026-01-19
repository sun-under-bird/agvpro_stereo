#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "iostream"
using namespace std::chrono;
class StereoCameraNode:public rclcpp::Node{
    public:
    StereoCameraNode():Node("stereo_camera_node")
    {
        //parameter
        this->declare_parameter<int>("right_cam_idx",2);
        this->declare_parameter<int>("width",640);
        this->declare_parameter<int>("height",480);


        //get parameter
        int right_idx=this->get_parameter("right_cam_idx").as_int();
        int width=this->get_parameter("width").as_int();
        int height=this->get_parameter("height").as_int();


        //read image
        cap_right_.open(right_idx, cv::CAP_V4L2);

        RCLCPP_INFO(this->get_logger(),"aaa");
        if(!cap_right_.isOpened()){
            RCLCPP_FATAL(this->get_logger(),"Check /dev/video index");
            rclcpp::shutdown();
            return;
        }

        //width and height
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH,width);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT,height);

        RCLCPP_INFO(this->get_logger(),"aaa");
        cap_right_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
 

        RCLCPP_INFO(this->get_logger(),"aaa");
        //Publisher
        pub_right_=this->create_publisher<sensor_msgs::msg::Image>("/right_camera/image_raw",10);
 
        timer_=this->create_wall_timer(milliseconds(33),std::bind(&StereoCameraNode::stereo_callback,this));

        RCLCPP_INFO(this->get_logger(), "Simple stereo camera node started successfully!");
        
    }

    private:
    void stereo_callback(){
        static int frame_count = 0;
        if (frame_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Callback triggered %d times (≈1 second)", frame_count);
        }
        frame_count++;
        cv::Mat frame_right;
        cap_right_>>frame_right;
        if (frame_right.empty())
        {
            RCLCPP_WARN_ONCE(this->get_logger(),"Empty frame");
            return;
        }

        //To ros
        auto msg_right=cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame_right
        ).toImageMsg();


        msg_right->header.stamp=this->now();


        msg_right->header.frame_id="right_camera";
   

        //Publish
        pub_right_->publish(*msg_right);
 
    }
    cv::VideoCapture cap_right_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}