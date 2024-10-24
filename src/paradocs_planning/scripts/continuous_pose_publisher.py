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
        # self.subsciber = self.create_subscription(Bool, 'test/start', self.listener_callback, 10)
        
        # self.start_publishing = False

        self.marker_publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tracking_goal_publisher_ = self.create_publisher(PoseStamped, 'tracking_goal', 10)
        self.relative_to_frame = 'lbr/link_0'

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cnt = 1
        self.diff = 0.05
        self.smallDiff = 0.02
        self.cntMax = 10
        timer_period = 15.0  # seconds
        # timer_period = 0.2  # 5 Hz

        self.basePose = Pose()
        self.basePose.position.x = -0.4
        self.basePose.position.y = 0.0
        self.basePose.position.z = 0.36
        self.basePose.orientation.x = 0.0
        self.basePose.orientation.y = 1.0
        self.basePose.orientation.z = 0.0
        self.basePose.orientation.w = 0.0

        Clock().sleep_for(rclpy.duration.Duration(seconds=10))
        # call once and then start the timer
        self.timer_callback()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # def get_transform(self):
    #     try:
    #         # Get the transformation from 'link_0' to 'link_tool'
    #         transform = self.tf_buffer.lookup_transform('lbr/link_0', 'lbr/link_tool', rclpy.time.Time())
    #         return transform
    #     except Exception as e:
    #         self.get_logger().warn(f'Could not get transform: {str(e)}')

    def timer_callback(self):
        
        if self.cnt <= self.cntMax:
            self.cnt += 1
        else:
            return
        
        # Get the transform
        # transform = self.get_transform()

        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()

        goalPose = Pose()
        goalPose.position.x = self.basePose.position.x
        # if (self.cnt < 3):
        goalPose.position.y = self.basePose.position.y + self.diff * self.cnt
        # else:
        #    msg.pose.position.y = self.basePose.position.y + self.diff * 3 + self.smallDiff * (self.cnt - 3)
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