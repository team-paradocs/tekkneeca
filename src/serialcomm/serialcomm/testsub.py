import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class TestSub(Node):

    def __init__(self):
        super().__init__('testsub')
        self.subscription=self.create_subscription(
            String,
            'drill_commands',
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, msg):
        self.get_logger().info("Heard "+msg.data)


def main(args=None):

    rclpy.init()
    testsub=TestSub()
    rclpy.spin(testsub)

    rclpy.shutdown()

if __name__=='__main__':
    main()




