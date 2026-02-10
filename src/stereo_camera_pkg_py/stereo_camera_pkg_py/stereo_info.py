import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

# 定义一个辅助函数加载 YAML
def load_yaml_to_camerainfo(filename):
    with open(filename, 'r') as f:
        calib_data = yaml.safe_load(f)
    ci = CameraInfo()
    ci.width = calib_data['image_width']
    ci.height = calib_data['image_height']
    ci.distortion_model = calib_data['distortion_model']
    ci.d = calib_data['distortion_coefficients']['data']
    ci.k = calib_data['camera_matrix']['data']
    ci.r = calib_data['rectification_matrix']['data']
    ci.p = calib_data['projection_matrix']['data']
    return ci

class StereoDriverNode(Node):
    def __init__(self):
        super().__init__('stereo_driver_node')

        # === 1. 参数声明 ===
        self.declare_parameter('left_yaml_path', '')
        self.declare_parameter('right_yaml_path', '')
        
        left_yaml = self.get_parameter('left_yaml_path').value
        right_yaml = self.get_parameter('right_yaml_path').value

        # === 2. 加载标定文件 ===
        if not os.path.exists(left_yaml) or not os.path.exists(right_yaml):
            self.get_logger().error(f"标定文件未找到! 请检查路径:\nL: {left_yaml}\nR: {right_yaml}")
            # 这里如果不退出，后面会报错，为了演示简单，我们假设文件存在
        
        self.left_info_msg = load_yaml_to_camerainfo(left_yaml)
        self.right_info_msg = load_yaml_to_camerainfo(right_yaml)
        
        # === 3. 初始化工具 ===
        self.bridge = CvBridge()

        # === 4. 订阅原始大图 (640x240) ===
        # 注意：这里订阅 usb_cam 发出的 /image_raw
        self.sub_raw = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )

        # === 5. 创建发布者 ===
        # 左目
        self.pub_left_img = self.create_publisher(Image, '/stereo/left/camera/image_raw', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/stereo/left/camera/camera_info', 10)
        # 右目
        self.pub_right_img = self.create_publisher(Image, '/stereo/right/camera/image_raw', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/stereo/right/camera/camera_info', 10)

        self.get_logger().info("Stereo Driver Started: Cropping & Publishing Info...")

    def image_callback(self, msg):
        try:
            # 1. 将 ROS 图像转为 OpenCV 图像 (BGR格式)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 获取图像尺寸
            height, width, _ = cv_image.shape 
            mid_point = width // 2 

            # 3. 裁剪 (NumPy 切片操作，极快)
            # 左图：取左半边 [0:240, 0:320]
            left_frame = cv_image[0:height, 0:mid_point]
            # 右图：取右半边 [0:240, 320:640]
            right_frame = cv_image[0:height, mid_point:width]

            # 4. 转回 ROS 消息
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")

            # 5. 同步 Header (关键步骤！)
            # 所有的输出消息必须使用原始消息的时间戳
            timestamp = msg.header.stamp
            
            # 设置左眼 Header
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = "camera_left_frame"
            self.left_info_msg.header = left_msg.header

            # 设置右眼 Header
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = "camera_right_frame"
            self.right_info_msg.header = right_msg.header

            # 6. 发布所有话题
            self.pub_left_img.publish(left_msg)
            self.pub_left_info.publish(self.left_info_msg)
            
            self.pub_right_img.publish(right_msg)
            self.pub_right_info.publish(self.right_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
