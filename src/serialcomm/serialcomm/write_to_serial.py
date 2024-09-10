import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import serial
arduino=serial.Serial('/dev/ttyACM0', 9600)


class TestSub(Node):

    def __init__(self):
        super().__init__('serial_writer')
        self.subscription=self.create_subscription(
            String,
            '/lbr/drill_commands',
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, msg):
        self.get_logger().info("Heard "+msg.data)
        arduino.write(msg.data.encode())
        arduino.write(msg.data.encode())
        arduino.write(msg.data.encode())
        self.get_logger().info(arduino.readline())


def main(args=None):

    rclpy.init()
    testsub=TestSub()
    rclpy.spin(testsub)

    rclpy.shutdown()

if __name__=='__main__':
    main()




