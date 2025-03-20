#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseArray

class PoseTransformer(Node):

    def __init__(self):
        super().__init__('pose_transformer')
        self.publisher_ = self.create_publisher(PoseArray, 'transformed_surgical_drill_pose', 10)
        self.subscription = self.create_subscription(
            PoseArray,
            'surgical_drill_pose',
            self.listener_callback,
            10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        # Take first pose from array
        if len(msg.poses) > 0:
            pose = msg.poses[0]
            transform = self.tf_buffer.lookup_transform('lbr/link_0', msg.header.frame_id, rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            # Add offset in the x and y direction
            offset_x = 0.01  # example offset in meters
            offset_y = 0.03  # example offset in meters
            transformed_pose.position.x += offset_x
            transformed_pose.position.y -= offset_y
            
            # Convert to PoseArray
            pose_stamped_array = PoseArray()
            pose_stamped_array.header = msg.header
            pose_stamped_array.poses.append(transformed_pose)
            print("transformed_pose_array", pose_stamped_array)
            self.publisher_.publish(pose_stamped_array)

def main(args=None):
    rclpy.init(args=args)
    pose_transformer = PoseTransformer()
    rclpy.spin(pose_transformer)
    pose_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()