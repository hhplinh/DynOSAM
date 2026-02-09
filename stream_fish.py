cat << 'EOF' > stream_fish.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import time
import numpy as np

class FishStreamer(Node):
    def __init__(self):
        super().__init__('fish_streamer')
        self.publisher_rgb = self.create_publisher(Image, '/camera/color/image_rect_color', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.publisher_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 FPS
        self.bridge = CvBridge()
        
        # PATHS - Update these if needed
        self.base_path = '/root/data/sequences/00'
        self.rgb_path = os.path.join(self.base_path, 'image_2')
        self.depth_path = os.path.join(self.base_path, 'image_3') 
        
        self.images = sorted(os.listdir(self.rgb_path))
        self.idx = 0
        self.get_logger().info(f"Found {len(self.images)} images. Starting stream...")

    def timer_callback(self):
        if self.idx >= len(self.images):
            # Loop the video so the SLAM doesn't run out of data
            self.idx = 0 
        
        img_name = self.images[self.idx]
        
        # 1. Load RGB
        cv_img_rgb = cv2.imread(os.path.join(self.rgb_path, img_name))
        if cv_img_rgb is None: return
        
        # 2. Load Depth (Create dummy if missing)
        depth_file = os.path.join(self.depth_path, img_name)
        if os.path.exists(depth_file):
            cv_img_depth = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
        else:
            cv_img_depth = np.ones((cv_img_rgb.shape[0], cv_img_rgb.shape[1]), dtype=np.uint16) * 1000

        # 3. Create Messages
        timestamp = self.get_clock().now().to_msg()
        
        msg_rgb = self.bridge.cv2_to_imgmsg(cv_img_rgb, encoding="bgr8")
        msg_rgb.header.stamp = timestamp
        msg_rgb.header.frame_id = "camera_optical_frame"
        
        msg_depth = self.bridge.cv2_to_imgmsg(cv_img_depth, encoding="mono16")
        msg_depth.header.stamp = timestamp
        msg_depth.header.frame_id = "camera_optical_frame"

        # 4. Create Camera Info (CRITICAL UPDATE HERE)
        msg_info = CameraInfo()
        msg_info.header = msg_rgb.header
        msg_info.width = cv_img_rgb.shape[1]
        msg_info.height = cv_img_rgb.shape[0]
        
        # We tell DynoSAM we are using the "radtan" model with 0 distortion
        msg_info.distortion_model = "radtan"
        msg_info.d = [0.0, 0.0, 0.0, 0.0, 0.0] 
        
        msg_info.k = [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0] 
        msg_info.p = [615.0, 0.0, 320.0, 0.0, 0.0, 615.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        # 5. Publish
        self.publisher_rgb.publish(msg_rgb)
        self.publisher_depth.publish(msg_depth)
        self.publisher_info.publish(msg_info)
        
        print(f"Published frame {self.idx}: {img_name}")
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    streamer = FishStreamer()
    rclpy.spin(streamer)
    streamer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF