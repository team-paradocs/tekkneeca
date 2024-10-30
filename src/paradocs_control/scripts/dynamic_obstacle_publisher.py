import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np
from regpipe_ros.open3d_conversions import from_msg, to_msg
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

class DynamicObstaclePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_publisher')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/D435/depth/color/points',
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/workspace/cloud', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to Open3D PointCloud
        # points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        o3d_cloud = from_msg(msg)

        try:
            transform = self.tf_buffer.lookup_transform('lbr/link_0', msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            # print("transform error", e)
            return
        
        # o3d_cloud = o3d.geometry.PointCloud()
        # o3d_cloud.points = o3d.utility.Vector3dVector(points)

        # get the transoform matrix as a numpy array
        rot = transform.transform.rotation
        transformation_matrix = np.eye(4)
        # Convert quaternion to rotation matrix
        quaternion = [rot.x, rot.y, rot.z, rot.w]
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        trans = transform.transform.translation
        transformation_matrix[0, 3] = trans.x
        transformation_matrix[1, 3] = trans.y
        transformation_matrix[2, 3] = trans.z

        # Apply the transformation
        transformation_matrix[0:3, 0:3] = rotation_matrix
        o3d_cloud.transform(transformation_matrix)



        #show mw the point cloud
        # o3d.visualization.draw_geometries([o3d_cloud])

        # Crop the point cloud
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.9, -0.5, 0), max_bound=(-0.2, 0.5, 0.4))
        cropped_cloud = o3d_cloud.crop(bbox)

        # o3d.visualization.draw_geometries([cropped_cloud])

        # Convert Open3D PointCloud back to ROS PointCloud2
        # cropped_points = np.asarray(cropped_cloud.points)
        # header = msg.header
        cropped_msg = to_msg(cropped_cloud, 'lbr/link_0')

        # Publish the cropped point cloud
        self.publisher.publish(cropped_msg)
        # exit()

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()