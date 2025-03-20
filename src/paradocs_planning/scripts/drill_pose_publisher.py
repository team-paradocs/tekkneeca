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
        super().__init__('drill_pose_publisher')

        self.marker_publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tracking_goal_publisher_ = self.create_publisher(PoseStamped, 'tracked_pose', 10)
        self.relative_to_frame = 'lbr/link_0'

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 15.0  # seconds

        # for the teleoperated drill pose
        
        self.basePose = Pose()
        self.basePose.position.x = -0.4
        self.basePose.position.y = 0.2
        self.basePose.position.z = 0.36
        self.basePose.orientation.x = 0.6411
        self.basePose.orientation.y = 0.7623
        self.basePose.orientation.z = -0.0516
        self.basePose.orientation.w = 0.0724 

        Clock().sleep_for(rclpy.duration.Duration(seconds=5))
        # call once and then start the timer
        self.timer_callback()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()

        goalPose = Pose()
        goalPose.position.x = self.basePose.position.x
        goalPose.position.y = self.basePose.position.y
        goalPose.position.z = self.basePose.position.z
        goalPose.orientation.x = self.basePose.orientation.x
        goalPose.orientation.y = self.basePose.orientation.y
        goalPose.orientation.z = self.basePose.orientation.z
        goalPose.orientation.w = self.basePose.orientation.w

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

        # for hybrid-planning
        tracking_goal = PoseStamped()
        tracking_goal.header.stamp.sec = sec
        tracking_goal.header.stamp.nanosec = nsec
        tracking_goal.header.frame_id = self.relative_to_frame
        tracking_goal.pose = goalPose
        self.tracking_goal_publisher_.publish(tracking_goal)

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

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()