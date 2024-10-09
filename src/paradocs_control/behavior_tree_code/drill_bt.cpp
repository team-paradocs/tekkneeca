#include <rclcpp/rclcpp.hpp>                // ROS 2 core functionality
#include "behaviortree_cpp/behavior_tree.h" // Behavior Tree functionality
#include "behaviortree_cpp/bt_factory.h"

#include "std_msgs/msg/string.hpp" // ROS 2 String message type
                                   // #include <serial/serial.h>                  // For Arduino communication

rclcpp::Node::SharedPtr node_;

class SerialWriterBTNode : public BT::SyncActionNode
{   
    private:
        int status = 0;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    public:
        SerialWriterBTNode(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {

            node_ = rclcpp::Node::make_shared("serial_writer_node");

            // Create a ROS 2 subscription to the topic "/lbr/drill_commands"
            subscription_ = node_->create_subscription<std_msgs::msg::String>(
                "/bt/drill_commands", 10, // The topic, queue size 10
                std::bind(&SerialWriterBTNode::subscriber_callback, this, std::placeholders::_1));
            status = 1;        
        }
        BT::NodeStatus tick() override
        {
            if (status == 1)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else if (status == 2)
            {
                return BT::NodeStatus::RUNNING;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }

        }
        void subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::cout << "Sending to arduino: " << msg->data.c_str() << std::endl;
            status = 2;
        }

}