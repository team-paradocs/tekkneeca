#!/usr/bin/env python
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import ctypes
import open3d as o3d

class PointCloudSaver(Node):

    def __init__(self):
        super().__init__('point_cloud_saver')
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


    def process_point_cloud(self, ros_point_cloud):
        xyz = []
        rgb = []

        # gen = pc2.read_points(ros_point_cloud, skip_nans=True)

        gen = pc2.read_points(ros_point_cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))

        for x in gen:
            # self.get_logger().info("In loop")
            color = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', color)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            # print(r/255.0, g/255.0, b/255.0)
            xyz.append([x[0], x[1], x[2]])
            rgb.append([r/225.0, g/255.0, b/255.0])
        

        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(np.array(xyz))
        out_pcd.colors = o3d.utility.Vector3dVector(np.array(rgb).astype(np.float64))
        o3d.io.write_point_cloud("/home/paradocs/cloud.ply", out_pcd)

        self.get_logger().info("Finish")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    point_cloud_saver = PointCloudSaver()
    rclpy.spin(point_cloud_saver)
    point_cloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()