#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "drill_bt.cpp"

using namespace BT;


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_node");

    BehaviorTreeFactory factory;

    factory.registerNodeType<SerialWriterBTNode>("SerialWriterBTNode");
    // factory.registerNodeType<NodeB>("NodeB");
    // factory.registerNodeType<NodeC>("NodeC");

    // Load the tree from the file bt_svd.xml
    auto tree = factory.createTreeFromFile("bt_svd.xml");

    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
