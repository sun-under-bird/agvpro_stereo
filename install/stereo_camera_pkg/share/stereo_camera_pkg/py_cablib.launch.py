import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import message_filters

class StereoSyncCombinedNode(Node):
    def __init__(self):
        super().__init__("stereo_sync_combined_node")
        self.bridge = CvBridge()
        
        # -------------------------- 1. 定时器采集+统一打时间戳 --------------------------
        # 相机设备号（根据实际情况修改：左相机0，右相机1）
        self.cap_left = cv2.VideoCapture(0)
        self.cap_right = cv2.VideoCapture(2)
        # 设置相机分辨率（可选，根据实际标定参数调整）
        self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # 原始图像发布器（定时器输出）
        self.left_img_pub = self.create_publisher(Image, "/left/image_raw", 10)
        self.right_img_pub = self.create_publisher(Image, "/right/image_raw", 10)
        self.left_info_pub = self.create_publisher(CameraInfo, "/left/camera_info", 10)
        self.right_info_pub = self.create_publisher(CameraInfo, "/right/camera_info", 10)
        
        # 定时器（30Hz采集，核心：同一时刻采集左右帧并打相同时间戳）
        self.timer = self.create_timer(1.0/30, self.timer_callback)

        # -------------------------- 2. 时间同步器校验+发布同步帧 --------------------------
        # 订阅定时器发布的原始图像
        self.left_img_sub = message_filters.Subscriber(self, Image, "/left_camera/image_raw")
        self.right_img_sub = message_filters.Subscriber(self, Image, "/right_camera/image_raw")
        self.left_info_sub = message_filters.Subscriber(self, CameraInfo, "/left_camera/camera_info")
        self.right_info_sub = message_filters.Subscriber(self, CameraInfo, "/right_camera/camera_info")
        
        # 时间同步器：容忍最大时间差10ms（slop=0.01），队列长度10
        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self.left_img_sub, self.right_img_sub, self.left_info_sub, self.right_info_sub],
            queue_size=10,
            slop=0.01
        )
        self.time_synchronizer.registerCallback(self.sync_callback)
        
        # 同步后图像发布器（送给stereo_image_proc/RTAB-Map）
        self.sync_left_pub = self.create_publisher(Image, "/left_camera/image_raw", 10)
        self.sync_right_pub = self.create_publisher(Image, "/right_camera/image_raw", 10)
        self.sync_left_info_pub = self.create_publisher(CameraInfo, "/left_camera/camera_info", 10)
        self.sync_right_info_pub = self.create_publisher(CameraInfo, "/right_camera/camera_info", 10)
        
        self.get_logger().info("双目同步节点已启动！")

    def timer_callback(self):
        """定时器回调：同一时刻采集左右图像，打相同时间戳"""
        # 同步采集左右帧
        ret_l, frame_l = self.cap_left.read()
        ret_r, frame_r = self.cap_right.read()
        
        if not ret_l or not ret_r:
            self.get_logger().warn("相机采集失败，请检查设备号！")
            return
        
        # 核心：获取统一的系统时间戳（所有帧共用）
        current_stamp = self.get_clock().now().to_msg()
        
        # 发布左图像（带统一时间戳）
        img_msg_l = self.bridge.cv2_to_imgmsg(frame_l, "bgr8")
        img_msg_l.header.stamp = current_stamp
        img_msg_l.header.frame_id = "left_camera"
        self.left_img_pub.publish(img_msg_l)
        
        # 发布右图像（同时间戳）
        img_msg_r = self.bridge.cv2_to_imgmsg(frame_r, "bgr8")
        img_msg_r.header.stamp = current_stamp
        img_msg_r.header.frame_id = "right_camera"
        self.right_img_pub.publish(img_msg_r)
        
        # 发布camera_info（需替换为你的实际标定参数！）
        # 左相机camera_info
        left_info = CameraInfo()
        left_info.header.stamp = current_stamp
        left_info.header.frame_id = "left_camera"
        left_info.width, left_info.height = 320, 240
        left_info.k = [251.46443, 0.0, 162.2194, 0.0, 253.67909, 119.15555, 0.0, 0.0, 1.0]
        # 畸变系数D（distortion_coefficients的data）
        left_info.d = [0.147505, -0.410895, 0.004186, 0.002758, 0.000000]
        # 投影矩阵P（projection_matrix的data）
        left_info.p = [642.23833, 0.0, 210.49113, 0.0, 0.0, 642.23833, 107.63846, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.left_info_pub.publish(left_info)
        
        # 右相机camera_info（同理替换为实际参数）
        right_info = CameraInfo()
        right_info.header.stamp = current_stamp
        right_info.header.frame_id = "right_camera"
        right_info.width, right_info.height = 320, 240
        right_info.k = [287.31149, 0.0, 159.23116, 0.0, 288.8347, 113.67787, 0.0, 0.0, 1.0]
        # 畸变系数D（distortion_coefficients的data）
        right_info.d = [0.086574, -0.272178, -0.003424, 0.002781, 0.000000]
        # 投影矩阵P（projection_matrix的data，注意Tx=-53.92717）
        right_info.p = [642.23833, 0.0, 210.49113, -53.92717, 0.0, 642.23833, 107.63846, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.right_info_pub.publish(right_info)

    def sync_callback(self, left_img, right_img, left_info, right_info):
        """时间同步器回调：仅发布时间戳匹配的帧"""
        # 发布同步后的图像和camera_info（送给后续处理）
        self.sync_left_pub.publish(left_img)
        self.sync_right_pub.publish(right_img)
        self.sync_left_info_pub.publish(left_info)
        self.sync_right_info_pub.publish(right_info)
        
        # 可选：打印同步信息，验证效果
        self.get_logger().debug(f"同步成功！左帧时间戳：{left_img.header.stamp.sec}.{left_img.header.stamp.nanosec}")

    def __del__(self):
        """销毁节点时释放相机资源"""
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = StereoSyncCombinedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()