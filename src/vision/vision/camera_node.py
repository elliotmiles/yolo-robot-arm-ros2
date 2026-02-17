import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(
            Image,
            '/camera',
            10
        )


        self.timer_ = self.create_timer(
            # 30 FPS
            (1.0/30.0),  
            self.timer_callback
        ) 

        self.cap_ = cv.VideoCapture(0) 
        self.bridge_ = CvBridge()

        self.get_logger().info("Camera node started")


    def timer_callback(self):
        ret, frame = self.cap_.read()
        if not ret:
            self.get_logger().error('Failed to capture image from camera')
            return
        
        msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.cap_.release() 
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()