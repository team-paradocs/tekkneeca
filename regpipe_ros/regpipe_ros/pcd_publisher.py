#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d
from . import open3d_conversions



class PCDPublisher(Node):
    """A class for publishing point cloud data from a PCD file in ROS2.

    Attributes:
        file_name (str): The name of the PCD file to publish.
        topic (str): The name of the ROS2 topic to publish the PointCloud2 message on.
        camera_frame (str): The name of the camera frame to use for the PointCloud2 message.
        publisher_ (rclpy.publisher.Publisher): The ROS2 publisher for the PointCloud2 message.
    """

    def __init__(self):
        super().__init__("pcd_publisher")
        self.declare_parameter("pcd_file", "/home/warra/bone_dataset_1/ply/bone{}.ply".format(13))
        self.declare_parameter("pub_topic", "ply_pointcloud")
        self.declare_parameter("camera_frame", "camera_link")

        self.file_name = self.get_parameter("pcd_file").get_parameter_value().string_value
        self.topic = self.get_parameter("pub_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(PointCloud2, self.topic, 1)


        o3d_cloud = open3d.io.read_point_cloud(self.file_name)
        self.ply_file = self.file_name.split("/")[-1]

        self.get_logger().info(f"Publishing {self.ply_file} on topic {self.topic} wrt frame {self.camera_frame}")

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.o3d_cloud = o3d_cloud


    def timer_callback(self):
        pcd_file_param = self.get_parameter("pcd_file").get_parameter_value().string_value
        if self.file_name != pcd_file_param:
            self.file_name = pcd_file_param
            self.o3d_cloud = open3d.io.read_point_cloud(self.file_name)
            self.ply_file = self.file_name.split("/")[-1]
            self.get_logger().info(f"Publishing {self.ply_file} on topic {self.topic} wrt frame {self.camera_frame}")

        self.convert_and_publish(self.o3d_cloud)

    def convert_and_publish(self, o3d_cloud):
        ros_cloud = open3d_conversions.to_msg(o3d_cloud)
        ros_cloud.header.stamp = self.get_clock().now().to_msg()
        ros_cloud.header.frame_id = self.camera_frame
        self.publisher_.publish(ros_cloud)


def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
