#include <rclcpp/rclcpp.hpp>                // ROS 2 core functionality
#include "behaviortree_cpp/behavior_tree.h" // Behavior Tree functionality
#include "behaviortree_cpp/bt_factory.h"

#include "std_msgs/msg/string.hpp" // ROS 2 String message type
                                   // #include <serial/serial.h>                  // For Arduino communication

rclcpp::Node::SharedPtr node_;

class SerialWriterBTNode : public BT::SyncActionNode
{
public:
    // Constructor
    SerialWriterBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {

        // Initialize the Arduino serial communication
        // arduino_.setPort("/dev/ttyACM0");  // Set Arduino port
        // arduino_.setBaudrate(9600);        // Set baud rate
        // arduino_.open();                   // Open the serial connection
        std::cout << "Inside SerialWriterBTNode constructor" << std::endl;
        // Create a ROS 2 node to handle subscriptions
        node_ = rclcpp::Node::make_shared("serial_writer_node");

        // Create a ROS 2 subscription to the topic "/lbr/drill_commands"
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "/lbr/drill_commands", 10, // The topic, queue size 10
            std::bind(&SerialWriterBTNode::subscriber_callback, this, std::placeholders::_1));
    }

    // Behavior Tree node ports (input and output for BT nodes)
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("command")}; // Input port for command messages
    }

    // The tick function will be called by the Behavior Tree when this node is executed
    BT::NodeStatus tick() override
    {
        //TODO add some shi here
        // No specific action required here for now as the callback handles the message
        return BT::NodeStatus::SUCCESS; // Return SUCCESS after ticking
    }

    rclcpp::Node::SharedPtr get_node() const
    {
        return node_;
    }

    // private:
    // Callback function when a message is received from the topic
    void subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Log the message received from the topic
        // RCLCPP_INFO(node_->get_logger(), "Received: %s", msg->data.c_str());

        // Write the message to the Arduino
        // arduino_.write(msg->data.c_str());
        std::cout << "Sending to arduino: " << msg->data.c_str() << std::endl;

        // Log Arduino's response after writing the message
        // std::string response = arduino_.readline();  // Read response from Arduino
        // RCLCPP_INFO(node_->get_logger(), "Arduino response: %s", response.c_str());
    }

    // ROS 2 node for handling publishers/subscribers

    // ROS 2 subscription to listen to messages on a topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // Serial connection to Arduino
    // serial::Serial arduino_;
};

int main(int argc, char **argv)
{
    std::cout << "Starting Behavior Tree" << std::endl;
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a Behavior Tree factory
    BT::BehaviorTreeFactory factory;

    // Register the custom SerialWriterBTNode with the Behavior Tree factory
    factory.registerNodeType<SerialWriterBTNode>("SerialWriter");

    // Load the Behavior Tree from an XML file
    auto tree = factory.createTreeFromFile("/ros_ws/src/paradocs_control/behavior_tree_code/drill_bt.xml");

    // ROS 2 spin and Behavior Tree execution
    rclcpp::Rate loop_rate(10); // Set loop rate to 10 Hz
    while (rclcpp::ok())
    {
        tree.tickWhileRunning();  // Tick the root of the tree to run the nodes
        rclcpp::spin_some(node_); // Process ROS callbacks
        // rclcpp::spin(serial_writer_node->get_node());  // Process ROS callbacks
        loop_rate.sleep(); // Sleep to maintain loop rate
    }

    // Shutdown ROS 2 when done
    rclcpp::shutdown();
    node_.reset();
    return 0;
}
