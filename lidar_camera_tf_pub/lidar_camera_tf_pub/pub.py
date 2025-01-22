import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
import numpy as np
import tkinter as tk
import threading


class LiDARCameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_calibration')

        # Publisher for the transformation
        self.publisher_ = self.create_publisher(TransformStamped, 'lidar_camera_transform', 10)
        
        # Publisher for the dummy camera info
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, 'camera_info_test', 10)
        
        # Timer to periodically publish the transformation
        self.timer_ = self.create_timer(0.1, self.publish_transform)
        
        # Timer to periodically publish the dummy camera info
        self.camera_info_timer_ = self.create_timer(1.0, self.publish_camera_info)

        # Initial transformation parameters
        self.translation = [0.0, 0.0, 0.0]  # x, y, z
        self.rotation = [0.0, 0.0, 0.0]     # roll, pitch, yaw

        # Start Tkinter GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.create_gui, daemon=True)
        self.gui_thread.start()

    def create_gui(self):
        # Create a Tkinter root window
        self.root = tk.Tk()
        self.root.title("LiDAR-Camera Calibration")

        # Create sliders for translation and rotation
        self.sliders = {}
        self.create_slider("Tx", -5.0, 5.0, 0.0, 0)
        self.create_slider("Ty", -5.0, 5.0, 0.0, 1)
        self.create_slider("Tz", -5.0, 5.0, 0.0, 2)
        self.create_slider("Roll", -180, 180, 0.0, 3)
        self.create_slider("Pitch", -180, 180, 0.0, 4)
        self.create_slider("Yaw", -180, 180, 0.0, 5)

        # Run Tkinter mainloop
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def create_slider(self, label, from_, to, initial, row):
        # Create a label for the slider
        tk.Label(self.root, text=label).grid(row=row, column=0)

        # Create the slider
        slider = tk.Scale(self.root, from_=from_, to=to, resolution=0.1, orient=tk.HORIZONTAL)
        slider.set(initial)
        slider.grid(row=row, column=1)
        self.sliders[label] = slider

    def get_slider_values(self):
        # Update translation and rotation from sliders
        self.translation[0] = self.sliders["Tx"].get()
        self.translation[1] = self.sliders["Ty"].get()
        self.translation[2] = self.sliders["Tz"].get()
        self.rotation[0] = np.radians(self.sliders["Roll"].get())
        self.rotation[1] = np.radians(self.sliders["Pitch"].get())
        self.rotation[2] = np.radians(self.sliders["Yaw"].get())

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
        transform.header.frame_id = 'os_sensor'
        transform.child_frame_id = 'default_cam'

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

    def publish_camera_info(self):
        # Create a dummy CameraInfo message
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'default_cam'
        
        # Dummy intrinsic parameters
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Zero distortion
        # camera_info.k = [1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0]
        camera_info.k = [
            500.0, 0.0, 320.0,  # fx,  0, cx
            0.0, 500.0, 240.0,  #  0, fy, cy
            0.0, 0.0, 1.0        #  0,  0,  1
        ]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # camera_info.p = [1.0, 0.0, 320.0, 0.0, 0.0, 1.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info.p = [
            500.0, 0.0, 320.0, 0.0,  # fx,  0, cx, Tx
            0.0, 500.0, 240.0, 0.0,  #  0, fy, cy, Ty
            0.0, 0.0, 1.0, 0.0        #  0,  0,  1,  0
        ]

        # Publish the dummy camera info
        self.camera_info_publisher_.publish(camera_info)

    def on_close(self):
        # Handle GUI close event
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()


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


if __name__ == '__main__':
    main()
