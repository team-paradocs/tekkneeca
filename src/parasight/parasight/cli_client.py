import questionary
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class ParaSightNode(Node):
    def __init__(self):
        super().__init__('parasight_cli_node')
        # Publishers for the topics
        self.start_publisher = self.create_publisher(Empty, 'trigger_host_ui', 10)
        self.stop_publisher = self.create_publisher(Empty, 'hard_reset_host', 10)
        self.drill_publisher = self.create_publisher(Empty, 'trigger_drill', 10)

    def publish_start(self):
        # Publish an empty message to start topic
        self.start_publisher.publish(Empty())
        self.get_logger().info("Published Start message on 'trigger_host_ui'")

    def publish_stop(self):
        # Publish an empty message to stop topic
        self.stop_publisher.publish(Empty())
        self.get_logger().info("Published Stop message on 'hard_reset_host'")

    def publish_drill(self):
        self.drill_publisher.publish(Empty())
        self.get_logger().info("Published Drill message on 'trigger_drill'")

class ParaSightCLI:
    def __init__(self):
        # Initialize ROS2 and the node
        rclpy.init()
        self.node = ParaSightNode()

    def run(self):
        while True:
            # Show the CLI menu and get the user's choice
            choice = questionary.select(
                "Select an option:",
                choices=[
                    "Start ParaSight",
                    "Reset ParaSight",
                    "Drill",
                    "Exit"
                ]
            ).ask()

            if choice == "Start ParaSight":
                self.node.publish_start()
            elif choice == "Reset ParaSight":
                self.node.publish_stop()
            elif choice == "Drill":
                self.node.publish_drill()
            elif choice == "Exit":
                print("Exiting...")
                break

        # Shutdown ROS2 once the loop is exited
        rclpy.shutdown()

def main():
    cli = ParaSightCLI()
    cli.run()

if __name__ == "__main__":
    main()