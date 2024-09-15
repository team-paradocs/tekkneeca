#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 5.0  # seconds
        # timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.base_pose = Pose()
        self.base_pose.position.x = -0.4
        self.base_pose.position.y = 0.0
        self.base_pose.position.z = 0.36
        self.base_pose.orientation.x = 0.0
        self.base_pose.orientation.y = 1.0
        self.base_pose.orientation.z = 0.0
        self.base_pose.orientation.w = 0.0

    def timer_callback(self):
        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()

        msg = InteractiveMarkerFeedback()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = 'world'
        msg.client_id = "/rviz2"
        msg.marker_name = "Goal"
        msg.control_name = ''
        msg.event_type = 2
        msg.pose.position.x = self.base_pose.position.x + random.uniform(-0.05, 0.05)
        msg.pose.position.y = self.base_pose.position.y + random.uniform(-0.05, 0.05)
        msg.pose.position.z = self.base_pose.position.z + random.uniform(-0.05, 0.05)
        msg.pose.orientation.x = self.base_pose.orientation.x + random.uniform(-0.05, 0.05)
        msg.pose.orientation.y = self.base_pose.orientation.y + random.uniform(-0.05, 0.05)
        msg.pose.orientation.z = self.base_pose.orientation.z + random.uniform(-0.05, 0.05)
        msg.pose.orientation.w = self.base_pose.orientation.w + random.uniform(-0.05, 0.05)

        # msg.pose.position.x = self.base_pose.position.x
        # msg.pose.position.y = self.base_pose.position.y
        # msg.pose.position.z = self.base_pose.position.z
        # msg.pose.orientation.x = self.base_pose.orientation.x
        # msg.pose.orientation.y = self.base_pose.orientation.y
        # msg.pose.orientation.z = self.base_pose.orientation.z
        # msg.pose.orientation.w = self.base_pose.orientation.w

        msg.menu_entry_id = 1
        msg.mouse_point_valid = True
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

        # Also publish the transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'command'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
