import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'tracked_pose', 10)
        self.get_logger().info('Press Enter to publish a PoseStamped message...')

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'lbr/link_0'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = -0.45
        pose.pose.position.y = -0.05
        pose.pose.position.z = -0.0125
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        self.publisher_.publish(pose)
        self.get_logger().info('Published PoseStamped message')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    try:
        while rclpy.ok():
            input()  # Wait for Enter key press
            node.publish_pose()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()