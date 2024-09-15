#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import random
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import tf2_ros
import numpy as np
# Import necessary modules

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(InteractiveMarkerFeedback, 'simple_marker/feedback', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # timer_period = 5
        # timer_period = 0.2  # 5 Hz
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            PoseStamped,
            'aruco_single/pose',
            self.listener_callback,
            10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def quaternion_to_matrix(self, quat):
        x, y, z, w = quat
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w, 0],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2, 0],
            [0, 0, 0, 1]
        ])
    
    def quaternion_multiply(self, q1, q2):
        # q1 and q2 should be arrays of shape [4]
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        x = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2
        y = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2
        z = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2
        w = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2
        return np.array([x, y, z, w])

    
    def move_along_z(self, msg_pose, distance):

        # Create a translation along the Z axis in the frame of the given pose
        translation = np.array([0, 0, distance])

        # Convert Quaternion to a rotation matrix
        quaternion = [
            msg_pose.orientation.x,
            msg_pose.orientation.y,
            msg_pose.orientation.z,
            msg_pose.orientation.w
        ]
        rotation_matrix = self.quaternion_to_matrix(quaternion)

        # Apply the rotation to the translation vector
        translated_vector = np.dot(rotation_matrix, np.append(translation, 1))[:3]

        # Update the pose position
        new_pose = Pose()
        new_pose.position.x = msg_pose.position.x + translated_vector[0]
        new_pose.position.y = msg_pose.position.y + translated_vector[1]
        new_pose.position.z = msg_pose.position.z + translated_vector[2]
        new_pose.orientation = msg_pose.orientation

        return new_pose

    def listener_callback(self, msg):
        pose = Pose()
        pose.position = msg.pose.position
        pose.orientation = msg.pose.orientation
        # self.publisher_.publish(pose)
        # try:
        # frames = self.tf_buffer.all_frames_as_string()
        # print(frames)
        # transform = self.tf_buffer.lookup_transform('lbr/link_0', 'camera_depth_optical_frame', rclpy.time.Time())
        transform = self.tf_buffer.lookup_transform('lbr/link_0', msg.header.frame_id, rclpy.time.Time())
        # print("transofmr", transform)
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        transformed_pose = self.move_along_z(transformed_pose, 0.3)

        quaternion = [
            transformed_pose.orientation.x,
            transformed_pose.orientation.y,
            transformed_pose.orientation.z,
            transformed_pose.orientation.w
        ]
        x, y, z, w = self.quaternion_multiply(quaternion, [0.7071068, 0.7071068, 0, 0])
        transformed_pose.orientation.x = x
        transformed_pose.orientation.y = y
        transformed_pose.orientation.z = z
        transformed_pose.orientation.w = w


        # print("transformed_pose", transformed_pose)
        # transformed_pose.orientation.x = 0.0
        # transformed_pose.orientation.y = 1.0
        # transformed_pose.orientation.z = 0.0
        # transformed_pose.orientation.w = 0.0

        # print("after", transformed_pose)
        # Convert PoseStamped to Pose before publishing
        # inp = input("publish?")
        # if inp == 'Y' or inp=='y':
        #     self.publisher_.publish(transformed_pose)

        
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     self.get_logger().info('Transform error')

        feedback_msg = InteractiveMarkerFeedback()
        feedback_msg.header.stamp.sec = 0
        feedback_msg.header.stamp.nanosec = 0
        feedback_msg.header.frame_id = 'world'
        feedback_msg.client_id = "/rviz2"
        feedback_msg.marker_name = "Goal"
        feedback_msg.control_name = ''
        feedback_msg.event_type = 2
        feedback_msg.pose.position.x = transformed_pose.position.x
        feedback_msg.pose.position.y = transformed_pose.position.y
        feedback_msg.pose.position.z = transformed_pose.position.z
        feedback_msg.pose.orientation.x = transformed_pose.orientation.x
        feedback_msg.pose.orientation.y = transformed_pose.orientation.y
        feedback_msg.pose.orientation.z = transformed_pose.orientation.z
        feedback_msg.pose.orientation.w = transformed_pose.orientation.w

        feedback_msg.menu_entry_id = 1
        feedback_msg.mouse_point_valid = True
        self.publisher_.publish(feedback_msg)

        # Also publish the transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'command'
        t.transform.translation.x = feedback_msg.pose.position.x
        t.transform.translation.y = feedback_msg.pose.position.y
        t.transform.translation.z = feedback_msg.pose.position.z
        t.transform.rotation.x = feedback_msg.pose.orientation.x
        t.transform.rotation.y = feedback_msg.pose.orientation.y
        t.transform.rotation.z = feedback_msg.pose.orientation.z
        t.transform.rotation.w = feedback_msg.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)



    # def timer_callback(self):
    #     msg = InteractiveMarkerFeedback()
    #     msg.header.stamp.sec = 0
    #     msg.header.stamp.nanosec = 0
    #     msg.header.frame_id = 'world'
    #     msg.client_id = "/rviz2"
    #     msg.marker_name = "Goal"
    #     msg.control_name = ''
    #     msg.event_type = 2
    #     msg.pose.position.x = self.base_pose.position.x + random.uniform(-0.01, 0.01)
    #     msg.pose.position.y = self.base_pose.position.y + random.uniform(-0.01, 0.01)
    #     msg.pose.position.z = self.base_pose.position.z + random.uniform(-0.01, 0.01)
    #     msg.pose.orientation.x = self.base_pose.orientation.x + random.uniform(-0.05, 0.05)
    #     msg.pose.orientation.y = self.base_pose.orientation.y + random.uniform(-0.05, 0.05)
    #     msg.pose.orientation.z = self.base_pose.orientation.z + random.uniform(-0.05, 0.05)
    #     msg.pose.orientation.w = self.base_pose.orientation.w + random.uniform(-0.05, 0.05)

    #     # msg.pose.position.x = self.base_pose.position.x
    #     # msg.pose.position.y = self.base_pose.position.y
    #     # msg.pose.position.z = self.base_pose.position.z
    #     # msg.pose.orientation.x = self.base_pose.orientation.x
    #     # msg.pose.orientation.y = self.base_pose.orientation.y
    #     # msg.pose.orientation.z = self.base_pose.orientation.z
    #     # msg.pose.orientation.w = self.base_pose.orientation.w

    #     msg.menu_entry_id = 1
    #     msg.mouse_point_valid = True
    #     self.publisher_.publish(msg)
    #     # self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# #!/usr/bin/env python
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import PoseStamped
# import tf2_ros
# import tf2_geometry_msgs

# class PoseTransformer(Node):

#     def __init__(self):
#         super().__init__('aruco_pose_transformer')
#         self.publisher_ = self.create_publisher(Pose, 'lbr/moveit_goal', 10)
#         self.subscription = self.create_subscription(
#             PoseStamped,
#             'aruco_single/pose',
#             self.listener_callback,
#             10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#     def listener_callback(self, msg):
#         pose = Pose()
#         pose.position = msg.pose.position
#         pose.orientation = msg.pose.orientation
#         # self.publisher_.publish(pose)
#         # try:
#         frames = self.tf_buffer.all_frames_as_string()
#         print(frames)
#         # transform = self.tf_buffer.lookup_transform('lbr/link_0', 'camera_depth_optical_frame', rclpy.time.Time())
#         transform = self.tf_buffer.lookup_transform('lbr/link_0', msg.header.frame_id, rclpy.time.Time())
#         print("transofmr", transform)
#         transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
#         print("transformed_pose", transformed_pose)
#         transformed_pose.orientation.x = 0.0
#         transformed_pose.orientation.y = 1.0
#         transformed_pose.orientation.z = 0.0
#         transformed_pose.orientation.w = 0.0


#         print("after", transformed_pose)
#         # Convert PoseStamped to Pose before publishing
#         inp = input("publish?")
#         if inp == 'Y' or inp=='y':
#             self.publisher_.publish(transformed_pose)

        
#         # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         #     self.get_logger().info('Transform error')

# def main(args=None):
#     rclpy.init(args=args)
#     pose_transformer = PoseTransformer()
#     rclpy.spin(pose_transformer)
#     pose_transformer.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()