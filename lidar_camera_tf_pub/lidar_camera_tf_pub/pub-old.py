import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2

class LiDARCameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_calibration')
        
        # Publisher for the transformation
        self.publisher_ = self.create_publisher(TransformStamped, 'lidar_camera_transform', 10)
        
        # Timer to periodically publish the transformation
        self.timer_ = self.create_timer(0.1, self.publish_transform)
        
        # Initial transformation parameters
        self.translation = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.rotation = np.array([0.0, 0.0, 0.0])     # roll, pitch, yaw
        
        # OpenCV sliders for manual adjustment
        self.create_sliders()
        
    def create_sliders(self):
        cv2.namedWindow('Calibration Sliders')
        
        cv2.createTrackbar('Tx', 'Calibration Sliders', 50, 100, lambda x: None)
        cv2.createTrackbar('Ty', 'Calibration Sliders', 50, 100, lambda x: None)
        cv2.createTrackbar('Tz', 'Calibration Sliders', 50, 100, lambda x: None)
        cv2.createTrackbar('Roll', 'Calibration Sliders', 50, 100, lambda x: None)
        cv2.createTrackbar('Pitch', 'Calibration Sliders', 50, 100, lambda x: None)
        cv2.createTrackbar('Yaw', 'Calibration Sliders', 50, 100, lambda x: None)
        
    def get_slider_values(self):
        # Map slider values back to their ranges
        self.translation[0] = (cv2.getTrackbarPos('Tx', 'Calibration Sliders') - 50) / 10.0
        self.translation[1] = (cv2.getTrackbarPos('Ty', 'Calibration Sliders') - 50) / 10.0
        self.translation[2] = (cv2.getTrackbarPos('Tz', 'Calibration Sliders') - 50) / 10.0
        self.rotation[0] = (cv2.getTrackbarPos('Roll', 'Calibration Sliders') - 50) * np.pi / 180
        self.rotation[1] = (cv2.getTrackbarPos('Pitch', 'Calibration Sliders') - 50) * np.pi / 180
        self.rotation[2] = (cv2.getTrackbarPos('Yaw', 'Calibration Sliders') - 50) * np.pi / 180
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return (qx, qy, qz, qw)
        
    def publish_transform(self):
        # Update values from sliders
        self.get_slider_values()
        
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'lidar_frame'
        transform.child_frame_id = 'camera_frame'
        
        # Translation
        transform.transform.translation.x = self.translation[0]
        transform.transform.translation.y = self.translation[1]
        transform.transform.translation.z = self.translation[2]
        
        # Rotation (quaternion)
        qx, qy, qz, qw = self.euler_to_quaternion(*self.rotation)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        
        # Publish the transform
        self.publisher_.publish(transform)
        
        # Show current transformation in the terminal
        self.get_logger().info(
            f'Translation: {self.translation}, Rotation: {np.degrees(self.rotation)} degrees'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LiDARCameraCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
