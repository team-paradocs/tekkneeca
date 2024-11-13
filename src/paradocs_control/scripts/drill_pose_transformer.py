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
        self.publisher_ = self.create_publisher(PoseStamped, 'lbr/moveit_goal', 10)
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
            
            # Convert to PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = transformed_pose
            print("transformed_pose", pose_stamped)
            self.publisher_.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    pose_transformer = PoseTransformer()
    rclpy.spin(pose_transformer)
    pose_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()