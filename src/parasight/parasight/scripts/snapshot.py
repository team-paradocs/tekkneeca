"""
This script is used to take a snapshot of the data coming on ROS2 topics and save it to a file.

Currenty saves,
 - RGB Image : /camera/color/image_rect_raw
 - Depth Image : /camera/depth/image_rect_raw
 - PointCloud : /camera/depth/color/points
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import os
from parasight.utils import from_msg as cloud_from_msg
import open3d as o3d

class SnapshotNode(Node):
    def __init__(self):
        super().__init__('snapshot_node')
        
        self.bridge = CvBridge()
        self.counter = 0
        
        # Store latest messages
        self.latest_rgb = None
        self.latest_depth = None 
        self.latest_cloud = None
        
        # Create data directory if it doesn't exist
        os.makedirs('src/data', exist_ok=True)
        
        # Subscribe to topics
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.rgb_callback,
            10)
            
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
            
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.cloud_callback,
            10)
            
        self.get_logger().info("Press Enter to take a snapshot...")
            
    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
    def cloud_callback(self, msg):
        self.latest_cloud = cloud_from_msg(msg)
        
    def save_snapshot(self):
        if all([self.latest_rgb is not None,
               self.latest_depth is not None,
               self.latest_cloud is not None]):
               
            # Save RGB image
            rgb_filename = f'src/data/rgb_{self.counter:04d}.png'
            cv2.imwrite(rgb_filename, self.latest_rgb)
            
            # Save depth image
            depth_filename = f'src/data/depth_{self.counter:04d}.npz'
            np.savez_compressed(depth_filename, depth=self.latest_depth)
            
            # Save pointcloud
            cloud_filename = f'src/data/cloud_{self.counter:04d}.ply'
            o3d.io.write_point_cloud(cloud_filename, self.latest_cloud)
            
            self.get_logger().info(f"Saved snapshot {self.counter}")
            self.counter += 1
        else:
            self.get_logger().warn("Missing data, cannot save snapshot")

def main():
    rclpy.init()
    node = SnapshotNode()
    
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if input() == '':
                node.save_snapshot()
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()