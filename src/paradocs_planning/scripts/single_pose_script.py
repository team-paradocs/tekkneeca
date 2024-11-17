import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/surgical_drill_pose', 10)
        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        # Set header
        msg.header = Header()
        msg.header.stamp.sec = 1712867143
        msg.header.stamp.nanosec = 806493768
        msg.header.frame_id = 'camera_depth_optical_frame'
        
        # Set position
        msg.pose.position.x = -0.031465664852436584
        msg.pose.position.y = -0.08847891310859848
        msg.pose.position.z = 0.1801937155336791
        
        # Set orientation
        msg.pose.orientation.x = -0.3073725086576684
        msg.pose.orientation.y = 0.07565928687230863
        msg.pose.orientation.z = 0.8903468078046592
        msg.pose.orientation.w = -0.32723137848244066

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Published PoseStamped message")
        
        # Shutdown node after publishing once
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)

if __name__ == '__main__':
    main()
