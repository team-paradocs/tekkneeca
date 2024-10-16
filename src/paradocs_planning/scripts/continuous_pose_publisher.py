#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import random
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.subsciber = self.create_subscription(Bool, 'test/start', self.listener_callback, 10)
        
        self.start_publishing = False

        self.marker_publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tracking_goal_publisher_ = self.create_publisher(Pose, 'tracking_goal', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cnt = 1
        self.diff = 0.05
        self.cntMax = 10
        timer_period = 10.0  # seconds
        # timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.basePose = Pose()
        self.basePose.position.x = -0.4
        self.basePose.position.y = 0.0
        self.basePose.position.z = 0.36
        self.basePose.orientation.x = 0.0
        self.basePose.orientation.y = 1.0
        self.basePose.orientation.z = 0.0
        self.basePose.orientation.w = 0.0

    def get_transform(self):
        try:
            # Get the transformation from 'link_0' to 'link_tool'
            transform = self.tf_buffer.lookup_transform('lbr/link_0', 'lbr/link_tool', rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')


    def listener_callback(self, msg):
        self.get_logger().info(f'Received boolean: {msg.data}')
        self.start_publishing = msg.data
        self.timer_callback()

    def timer_callback(self):

        if not self.start_publishing:
            return
        
        if self.cnt <= self.cntMax:
            self.cnt += 1
        else:
            return
        
        # Get the transform
        # transform = self.get_transform()

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

        msg.pose.position.x = self.basePose.position.x
        msg.pose.position.y = self.basePose.position.y + self.diff * self.cnt
        msg.pose.position.z = self.basePose.position.z
        msg.pose.orientation.x = self.basePose.orientation.x
        msg.pose.orientation.y = self.basePose.orientation.y
        msg.pose.orientation.z = self.basePose.orientation.z
        msg.pose.orientation.w = self.basePose.orientation.w

        self.get_logger().info(f'Publishing pose: {msg.pose}')

        msg.menu_entry_id = 1
        msg.mouse_point_valid = True
        self.marker_publisher_.publish(msg)

        # Also publish the pose
        self.tracking_goal_publisher_.publish(msg.pose)

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