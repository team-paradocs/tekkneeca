import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
 
class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/lbr/moveit_goal', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'
 
        # Create a sample pose
        pose = Pose()
        # pose.position.x = -0.4744262721931818
        # pose.position.y = -0.07868496514673545
        # pose.position.z = 0.2
        # pose.orientation.x = 0.0
        # pose.orientation.y = -1.0
        # pose.orientation.z = 0.0
        # pose.orientation.w = 0.0
        pose.position.x = -0.6235057666106665
        pose.position.y = -0.07433535174374437
        pose.position.z = 0.18922919147714384
        pose.orientation.x = -0.16650747810760247
        pose.orientation.y = -0.96965595010248
        pose.orientation.z = -0.03789573291822688
        pose.orientation.w = 0.17494716800135454



        # Add the pose to the pose array
        pose_array.poses.append(pose)
 
        self.publisher_.publish(pose_array)
        self.get_logger().info('Publishing: "%s"' % pose_array)
 
def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()