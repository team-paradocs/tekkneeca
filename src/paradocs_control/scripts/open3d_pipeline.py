#!/usr/bin/env python
import open3d_conversions
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node
import open3d as o3d

class Open3DPipeline(Node):

    def __init__(self):
        super().__init__('open_3d_pipeline')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.callback,
            1)

    def callback(self, ros_point_cloud):
        self.get_logger().info("Callback")
        self.process_point_cloud(ros_point_cloud)
        # Properly destroy the subscription after processing the first message
        self.destroy_subscription(self.subscription)
        self.subscription = None  # Prevents any accidental reuse
        self.get_logger().info("Finished processing point cloud. Exiting.")


    def process_point_cloud(self, ros_point_cloud):
        o3d_cloud = open3d_conversions.from_msg(ros_point_cloud)
        # ros_cloud2 = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
        o3d.io.write_point_cloud("/home/paradocs/cloud.ply", o3d_cloud)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    point_cloud_saver = Open3DPipeline()
    rclpy.spin(point_cloud_saver)
    point_cloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()