#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped
import random
import time
import copy
import math
import numpy as np
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__('realistic_pose_publisher')
        # self.subsciber = self.create_subscription(Bool, 'test/start', self.listener_callback, 10)
        
        # self.start_publishing = False

        self.marker_publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tracking_goal_publisher_ = self.create_publisher(PoseStamped, 'tracking_goal', 10)
        self.relative_to_frame = 'lbr/link_0'

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cnt = 1
        self.diff = 0.27
        self.smallDiff = 0.02
        self.startSmallDiff = 2
        self.cntMax = 10
        # timer_period = 0.2  # 5 Hz

        self.basePose = Pose()
        self.basePose.position.x = -0.4
        self.basePose.position.y = 0.2
        self.basePose.position.z = 0.36
        self.basePose.orientation.x = 0.6411
        self.basePose.orientation.y = 0.7623
        self.basePose.orientation.z = -0.0516
        self.basePose.orientation.w = 0.0724 

        self.homePose = Pose()
        self.homePose.position.x = -0.4
        self.homePose.position.y = 0.0
        self.homePose.position.z = 0.36
        self.homePose.orientation.x = 0.0
        self.homePose.orientation.y = 1.0
        self.homePose.orientation.z = 0.0
        self.homePose.orientation.w = 0.0

        Clock().sleep_for(rclpy.duration.Duration(seconds=20))
        # call once and then start the timer
        self.start_seq()

    def start_seq(self):
        
        # Get the transform
        # transform = self.get_transform()

        goalPose = Pose()
        goalPose = copy.deepcopy(self.basePose)
        self.get_logger().info("The realistic goal")
        self.publish_pose(goalPose)
        # make sure the robot arrived at the realistic goal
        Clock().sleep_for(rclpy.duration.Duration(seconds=5))

        goalPose.position = self.randomize_position(self.basePose.position, 0.01)
        # small position offset
        self.get_logger().info("Small offset in position")
        self.publish_pose(goalPose)
        Clock().sleep_for(rclpy.duration.Duration(seconds=1))

        goalPose.orientation = self.yaw_orientation(goalPose.orientation, 20)
        # small orientation offset
        self.get_logger().info("Small offset in orientation")
        self.publish_pose(goalPose)
        Clock().sleep_for(rclpy.duration.Duration(seconds=1))

        goalPose.position = self.randomize_position(self.basePose.position, 0.01)
        # small position offset
        self.get_logger().info("Small offset in position")
        self.publish_pose(goalPose)
        Clock().sleep_for(rclpy.duration.Duration(seconds=1))

        goalPose.orientation = self.yaw_orientation(self.basePose.orientation, -20)
        # small orientation offset
        self.get_logger().info("Small offset in orientation")
        self.publish_pose(goalPose)
        Clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # Home but then small offset
        self.get_logger().info("Home goal")
        self.publish_pose(self.homePose)
        Clock().sleep_for(rclpy.duration.Duration(seconds=2))
        self.homePose.position = self.randomize_position(self.homePose.position, 0.02)
        self.get_logger().info("Small offset Home goal")
        self.publish_pose(self.homePose)

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


    def randomize_position(self, position, mangnitude):
        current_position = np.array([position.x,
                                position.y,
                                position.z])
        perturbed_position = Pose().position
        perturbed_position.x = current_position[0] + np.random.uniform(-mangnitude, mangnitude)
        perturbed_position.y = current_position[1] + np.random.uniform(-mangnitude, mangnitude)
        perturbed_position.z = current_position[2] + np.random.uniform(-mangnitude, mangnitude)
        return perturbed_position
    
    def yaw_orientation(self, orientation, yaw_deg):
        # Convert yaw from degrees to radians
        yaw_rad = math.radians(yaw_deg)
        
        # Current orientation quaternion
        current_quaternion = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Create quaternion for the yaw rotation
        yaw_quaternion = np.array([
            0,  # No x rotation
            0,  # No y rotation
            math.sin(yaw_rad / 2),  # z component
            math.cos(yaw_rad / 2)   # w component
        ])
        
        # Perform quaternion multiplication to apply yaw
        new_quaternion = self.quaternion_multiply(current_quaternion, yaw_quaternion)
        
        # Normalize the resulting quaternion
        normalized_quaternion = self.normalize_quaternion(new_quaternion)
        
        # Return as a new Pose object with updated orientation
        perturbed_orientation = Pose().orientation
        perturbed_orientation.x = normalized_quaternion[0]
        perturbed_orientation.y = normalized_quaternion[1]
        perturbed_orientation.z = normalized_quaternion[2]
        perturbed_orientation.w = normalized_quaternion[3]
        
        return perturbed_orientation


    def randomize_orientation(self, orientation, mangnitude):
        # Convert current orientation (quaternion) to numpy array
        current_orientation = np.array([orientation.x,
                                        orientation.y,
                                        orientation.z,
                                        orientation.w])
        
        # Generate a small random rotation using quaternion
        random_quat = self.random_quaternion(mangnitude)  # 0.3 is the perturbation range
        
        # Multiply current quaternion by random quaternion (quaternion multiplication)
        new_quat = self.quaternion_multiply(current_orientation, random_quat)
        
        # Normalize the resulting quaternion to make it valid
        new_quat = self.normalize_quaternion(new_quat)
        
        # Convert back to geometry_msgs/Quaternion
        perturbed_orientation = Pose().orientation
        perturbed_orientation.x = new_quat[0]
        perturbed_orientation.y = new_quat[1]
        perturbed_orientation.z = new_quat[2]
        perturbed_orientation.w = new_quat[3]
        
        return perturbed_orientation

    def random_quaternion(self, perturbation_range):
        # Random axis
        rand_axis = np.random.randn(3)
        rand_axis /= np.linalg.norm(rand_axis)  # Normalize axis
        
        # Random angle in radians
        rand_angle = np.random.uniform(-perturbation_range, perturbation_range)
        
        # Convert angle-axis to quaternion
        w = math.cos(rand_angle / 2.0)
        x = rand_axis[0] * math.sin(rand_angle / 2.0)
        y = rand_axis[1] * math.sin(rand_angle / 2.0)
        z = rand_axis[2] * math.sin(rand_angle / 2.0)
        
        return np.array([x, y, z, w])
    
    def quaternion_multiply(self, q1, q2):
        # Perform quaternion multiplication (Hamilton product)
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([x, y, z, w])
    
    def normalize_quaternion(self, quat):
        # Normalize the quaternion
        norm = np.linalg.norm(quat)
        return quat / norm


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()