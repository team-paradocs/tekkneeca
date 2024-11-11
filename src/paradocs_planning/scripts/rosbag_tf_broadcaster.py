#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped
import random
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.bag_subsciber = self.create_subscription(PoseStamped, 'tracked_pose', self.listener_callback, 10)
        self.marker_publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)

        self.relative_to_frame = 'lbr/link_0'

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_pose(self, goalPose):

        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()
        self.get_logger().info(f'Publishing pose: {goalPose}')

        # for ocs2
        msg = InteractiveMarkerFeedback()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.relative_to_frame
        msg.client_id = "/rviz2"
        msg.marker_name = "Goal"
        msg.pose = goalPose
        msg.control_name = ''
        msg.event_type = 2
        msg.menu_entry_id = 1
        msg.mouse_point_valid = True
        self.marker_publisher_.publish(msg)

        # for tf
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'lbr/link_0'
        t.child_frame_id = 'command'
        t.transform.translation.x = goalPose.position.x
        t.transform.translation.y = goalPose.position.y
        t.transform.translation.z = goalPose.position.z
        t.transform.rotation = goalPose.orientation
        self.tf_broadcaster.sendTransform(t)

    def listener_callback(self, msg):
        self.publish_pose(msg.pose)

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()