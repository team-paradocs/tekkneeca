import questionary
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType


class ParaSightNode(Node):
    def __init__(self):
        super().__init__('parasight_cli_node')
        self.bones = ['Both', 'Femur', 'Tibia']
        self.bone_idx = 0

        # Publishers for the topics
        self.start_publisher = self.create_publisher(Empty, 'trigger_host_ui', 10)
        self.stop_publisher = self.create_publisher(Empty, 'hard_reset_host', 10)
        self.drill_publisher = self.create_publisher(Int32, '/lbr/plan_flag', 10)
        self.reg_request_publisher = self.create_publisher(Int32, '/trigger_reg', 10)
        self.log_request_publisher = self.create_publisher(Empty, '/log_request', 10)

    def publish_start(self):
        # Publish an empty message to start topic
        self.start_publisher.publish(Empty())
        self.get_logger().info("Published Start message on 'trigger_host_ui'")

    def publish_stop(self):
        # Publish an empty message to stop topic
        self.stop_publisher.publish(Empty())
        self.get_logger().info("Published Stop message on 'hard_reset_host'")

    def publish_drill(self):
        msg = Int32()
        msg.data = 2
        self.drill_publisher.publish(msg)
        self.get_logger().info("Published Drill message (2) on '/lbr/plan_flag'")
        self.publish_stop() # Reset ParaSight after drilling

    def publish_reg(self):
        msg = Int32()
        msg.data = 0
        self.reg_request_publisher.publish(msg)
        self.get_logger().info("Published Registration request message (0) on '/trigger_reg'")

    def publish_reg_ui(self):
        msg = Int32()
        msg.data = 1
        self.reg_request_publisher.publish(msg)
        self.get_logger().info("Published Registration request with UI message (1) on '/trigger_reg'")

    def publish_log_request(self):
        self.log_request_publisher.publish(Empty())
        self.get_logger().info("Requested log")

    def publish_toggle_bones(self):
        """Toggle the selected bones parameter on the ParaSightHost node."""
        # Create client to set parameter on parasight_host node
        param_client = self.create_client(SetParameters, '/parasight_host/set_parameters')

        # Wait for the service to be available
        if not param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Parameter service not available')
            return

        # Create a parameter request
        param = Parameter()
        param.name = 'selected_bones'
        param.value.type = ParameterType.PARAMETER_STRING
        
        # Increment bone_idx, wrapping around to 0 if needed
        self.bone_idx = (self.bone_idx + 1) % len(self.bones)

        param.value.string_value = self.bones[self.bone_idx].lower()

        request = SetParameters.Request()
        request.parameters = [param]

        # Call service asynchronously
        future = param_client.call_async(request)
        future.add_done_callback(self.parameter_set_callback)



    def parameter_set_callback(self, future):
        """Callback to handle the result of the parameter set request."""
        try:
            response = future.result()
            self.get_logger().info('Parameter set successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to set parameter: {e}')

class ParaSightCLI:
    def __init__(self):
        # Initialize ROS2 and the node
        rclpy.init()
        self.node = ParaSightNode()

    def run(self):
        while True:
            # Show the CLI menu and get the user's choice
            mode = self.node.bones[self.node.bone_idx]
            choice = questionary.select(
                f"Select an option (M:{mode}):",
                choices=[
                    "Start ParaSight",
                    "Reset ParaSight",
                    "Drill",
                    "Register",
                    "Register (+UI)",
                    "Log",
                    "Toggle Bones",
                    "Exit"
                ],
                use_shortcuts=True
            ).ask()

            if choice == "Start ParaSight":
                self.node.publish_start()
            elif choice == "Reset ParaSight":
                self.node.publish_stop()
            elif choice == "Drill":
                self.node.publish_drill()
            elif choice == "Register":
                self.node.publish_reg()
            elif choice == "Register (+UI)":
                self.node.publish_reg_ui()
            elif choice == "Log":
                self.node.publish_log_request()
            elif choice == "Toggle Bones":
                self.node.publish_toggle_bones()
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