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
        self.publisher_ = self.create_publisher(PoseArray, 'lbr/moveit_goal', 10)
        self.subscription = self.create_subscription(
            PoseArray,
            'surgical_drill_pose',
            self.listener_callback,
            10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg):
        transformed_poses = PoseArray()
        # poses = PoseArray()
        for pose in msg.poses:
            # pose.position = msg.pose.position
            # pose.orientation = msg.pose.orientation
            # self.publisher_.publish(pose)
            # try:
            # frames = self.tf_buffer.all_frames_as_string()
            # print(frames)
            # transform = self.tf_buffer.lookup_transform('lbr/link_0', 'camera_depth_optical_frame', rclpy.time.Time())
            transform = self.tf_buffer.lookup_transform('lbr/link_0', msg.header.frame_id, rclpy.time.Time())
            # print("transform", transform)
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            # print("after", transformed_pose)
            # Convert PoseStamped to Pose before publishing
            transformed_poses.poses.append(transformed_pose)
        transformed_poses.header = msg.header
        print("transformed_poses", transformed_poses)
        self.publisher_.publish(transformed_poses)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     self.get_logger().info('Transform error')

def main(args=None):
    rclpy.init(args=args)
    pose_transformer = PoseTransformer()
    rclpy.spin(pose_transformer)
    pose_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# #!/usr/bin/env python
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# class JointStatePublisher(Node):

#     def __init__(self):
#         super().__init__('playground_joint_state_publisher')
#         self.publisher_ = self.create_publisher(JointState, 'joint_states', 1)
#         self.subscription = self.create_subscription(
#             JointState,
#             '/lbr/joint_states',
#             self.listener_callback,
#             1)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     joint_state_publisher = JointStatePublisher()
#     rclpy.spin(joint_state_publisher)
#     joint_state_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose

# class GoalPublisher(Node):

#     def __init__(self):
#         super().__init__('goal_publisher')
#         self.publisher_ = self.create_publisher(Pose, 'lbr/moveit_goal', 10)
#         msg = Pose()
#         msg.position.x = -0.4
#         msg.position.y = -0.08
#         msg.position.z = 0.4
#         msg.orientation.x = 0.9063
#         msg.orientation.y = 0.0
#         msg.orientation.z = 0.0
#         msg.orientation.w = 0.4226
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     goal_publisher = GoalPublisher()
#     # rclpy.spin(goal_publisher)
#     goal_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()