#!/usr/bin/env python

import multiprocessing
import numpy as np
import threading

import open3d as o3d
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from scipy.spatial.transform import Rotation as R
import geometry_msgs.msg

from . import open3d_conversions, proc_pipeline, registration



class PCDRegPipe(Node):
    """A class for subscribing to a PointCloud2 message in ROS2 and processing it with Open3D.
    Attributes:
        topic (str): The name of the ROS2 topic to subscribe to for the PointCloud2 message.
        camera_frame (str): The name of the camera frame to use for the PointCloud2 message.
    """

    def __init__(self):
        super().__init__("pcd_regpipe")
        self.declare_parameter("sub_topic", "/camera/depth/color/points")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")

        self.topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.subscriber_ = self.create_subscription(PointCloud2, self.topic, self.callback, 10)
        self._publisher_ = self.create_publisher(PointCloud2, 'processed_point_cloud', 10)
        self.pose_publisher = self.create_publisher(PoseArray, 'surgical_drill_pose', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, 'bounding_box_marker_array', 10)

        self.x_thresh = 0.25
        self.y_thresh = 0.15
        self.z_thresh = 0.25
        package_dir = get_package_share_directory('regpipe_ros')
        source = "femur_shell.ply"
        self.source = o3d.io.read_point_cloud(package_dir + "/source/" + source)
        self.plan_path = package_dir + "/source/plan_config.yaml"
        self.plan = "plan2"


        self.proc_pipe = proc_pipeline.PointCloudProcessingPipeline(self.x_thresh, self.y_thresh, self.z_thresh)
        self.estimator = registration.Estimator('centroid')
        self.refiner = registration.Refiner('ransac_icp')

        self.last_cloud = None
        self.pcd_center = None
        self.get_logger().info(f"Subscribing to topic {self.topic}. Press Enter to register.")

        # Start a separate thread for the input to not block the ROS2 node
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def callback(self, msg):
        """Callback function for the subscriber.
        Args:
            msg (sensor_msgs.msg.PointCloud2): The incoming PointCloud2 message.
        """
        try:
            cloud = open3d_conversions.from_msg(msg)
            self.last_cloud = cloud
            # self.get_logger().info(f"Received point cloud with {len(cloud.points)} points.")
        except Exception as e:
            self.get_logger().error(f"Error converting message to point cloud: {e}")

    def wait_for_input(self):
        """Waits for the user input to register the latest received point cloud."""
        while rclpy.ok():
            input("Press Enter to register the latest point cloud...")
            self.process_point_cloud()

    def visualize_point_cloud(self):
        """Visualizes the last received point cloud in an Open3D window."""
        if self.last_cloud:
            # Start visualization in a separate process to not block ROS2 node
            vis_process = multiprocessing.Process(target=self._vis_process_target)
            vis_process.start()
            vis_process.join()
        else:
            self.get_logger().info("No point cloud has been received yet.")

    def _vis_process_target(self, cloud=None):
        """The target function for the multiprocessing process that handles visualization."""
        if cloud is None:
            cloud = self.last_cloud
        if cloud:
            o3d.visualization.draw_geometries([cloud], window_name="Point Cloud",
                                            zoom=0.57000000000000017,
                                            front=[0.086126891594714566, 0.27099417737370496, -0.9587201439282379],
                                            lookat=[-0.11521162012853672, -0.39411284983247913, 1.8123871758649917],
                                            up=[-0.0069065635673039305, -0.96211034045195976, -0.27257291166787834])
            

    def process_point_cloud(self):
        """Processes the last received point cloud with Open3D."""
        if self.last_cloud:
            self.pcd_center = self.last_cloud.get_center()
            cloud = self.proc_pipe.run(self.last_cloud)

            self.source_cloud = self.source.voxel_down_sample(voxel_size=0.003)
            self.target_cloud = cloud.voxel_down_sample(voxel_size=0.003)


            init_transformation = self.estimator.estimate(self.source_cloud, self.target_cloud)
            # init_transformation = np.eye(4)
            transform = init_transformation
            transform = self.refiner.refine(self.source_cloud, self.target_cloud, init_transformation)
            self.get_logger().info(f"Transformation matrix: {transform}")

            self.source_cloud.transform(transform)
            self.source_cloud.paint_uniform_color([1, 0, 0])

            self.publish_point_cloud(self.source_cloud)

            if_publish = input("Do you want to publish (y/n)?")

            if if_publish == "y":
                surgical_drill_pose = self.compute_plan(transform)
                self.pose_publisher.publish(surgical_drill_pose)
                print("Published Pose!")
            else:
                print("Not publishing. Moving on.")
                return

             

    def compute_plan(self, transform,theta=-np.pi):
        """Computes the surgical drill point by transforming the default point with the given transform."""

        drill_pose_array = PoseArray()
        drill_pose_array.header.frame_id = self.camera_frame
        drill_pose_array.header.stamp = self.get_clock().now().to_msg()

        holes = self.load_plan_points(self.plan)
        for hole_name, hole in holes.items():
            p1, p2, p3 = hole

            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector([p1, p2, p3])
            mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
            mesh.compute_vertex_normals()
            normal =  np.asarray(mesh.vertex_normals)[0]
            actual_normal = -normal
            z_axis = np.array([0, 0, 1])
            rotation_axis = np.cross(z_axis, actual_normal)
            rotation_axis /= np.linalg.norm(rotation_axis)
            angle = np.arccos(np.dot(z_axis, actual_normal) / (np.linalg.norm(z_axis) * np.linalg.norm(actual_normal)))
            Rot = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)

            T = np.eye(4)
            T[:3, :3] = Rot
            T[:3, 3] = p3

            default_plan = T
            default_plan_rotated = np.copy(default_plan)
            default_plan_rotated[:3, :3] = np.matmul(default_plan_rotated[:3, :3], R.from_euler('z', theta).as_matrix())


            T = np.matmul(transform, default_plan_rotated)


            # Convert R to a quaternion
            Rot = T[:3, :3]
            r = R.from_matrix(Rot)
            quat = r.as_quat()  # Returns (x, y, z, w)

            p = T[:3, 3]

            pose = PoseStamped()
            pose.header.frame_id = self.camera_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            self.get_logger().info(f"Drill point: {p}")
            drill_pose_array.poses.append(pose.pose)

        
        # print(drill_pose_array)
        return drill_pose_array
    
    def load_plan_points(self,plan_name):
        holes = {}
        with open(self.plan_path, 'r') as file:
            data = yaml.safe_load(file)  # Use safe_load to prevent arbitrary code execution
            plan_data = data.get(plan_name, {})
            # print(plan_data)
            for hole_name, hole in plan_data.items():
                for point_name, point_coords in hole.items():
                    if point_name == "p1":
                        p1 = np.array(point_coords) / 1000.0
                    elif point_name == "p2":
                        p2 = np.array(point_coords) / 1000.0
                    elif point_name == "p3":
                        p3 = np.array(point_coords) / 1000.0
                holes[hole_name] = [p1, p2, p3]

            # p1 = np.array(plan_data.get('p1', [])) / 1000.0
            # p2 = np.array(plan_data.get('p2', [])) / 1000.0
            # p3 = np.array(plan_data.get('p3', [])) / 1000.0
            return holes

            
    def publish_point_cloud(self, cloud):
        """Publishes the processed point cloud to a new ROS2 topic.
        Args:
            cloud (open3d.geometry.PointCloud): The processed point cloud to publish.
        """
        try:
            self.publish_bounding_box()
            msg = open3d_conversions.to_msg(cloud, frame_id=self.camera_frame)
            self._publisher_.publish(msg)
            self.get_logger().info(f"Processed point cloud with {len(cloud.points)} points published.")
        except Exception as e:
            self.get_logger().error(f"Error converting point cloud to message: {e}")

    def publish_bounding_box(self):
            """Publishes a marker array representing a 3D bounding box around the point cloud."""
            if not self.last_cloud:
                self.get_logger().info("No point cloud has been received to draw a bounding box.")
                return

            # Define corners of the bounding box based on the threshold values
            center = self.pcd_center
            x_thresh = self.x_thresh
            y_thresh = self.y_thresh
            z_thresh = self.z_thresh

            # Calculate corners of the bounding box
            corners = [
                center + np.array([x_thresh, y_thresh, z_thresh]),
                center + np.array([-x_thresh, y_thresh, z_thresh]),
                center + np.array([-x_thresh, -y_thresh, z_thresh]),
                center + np.array([x_thresh, -y_thresh, z_thresh]),
                center + np.array([x_thresh, y_thresh, -z_thresh]),
                center + np.array([-x_thresh, y_thresh, -z_thresh]),
                center + np.array([-x_thresh, -y_thresh, -z_thresh]),
                center + np.array([x_thresh, -y_thresh, -z_thresh])
            ]

            # Define edges of the bounding box
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # Upper face
                (4, 5), (5, 6), (6, 7), (7, 4),  # Lower face
                (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
            ]

            marker_array = MarkerArray()
            marker_id = 0

            for start, end in edges:
                marker = Marker()
                marker.header.frame_id = self.camera_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "bounding_box"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.points = [
                    geometry_msgs.msg.Point(x=corners[start][0], y=corners[start][1], z=corners[start][2]),
                    geometry_msgs.msg.Point(x=corners[end][0], y=corners[end][1], z=corners[end][2])
                ]
                marker.scale.x = 0.01  # Width of the line
                marker.color.a = 1.0  # Don't forget to set the alpha!
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker_array.markers.append(marker)
                marker_id += 1

            self.marker_array_publisher.publish(marker_array)
            self.get_logger().info("Bounding box marker array published.")


def main(args=None):
    rclpy.init(args=args)
    pcd_regpipe = PCDRegPipe()
    rclpy.spin(pcd_regpipe)
    pcd_regpipe.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()