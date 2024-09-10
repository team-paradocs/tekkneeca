#include <memory>

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// rclcpp::Logger* logger;

// moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declare a global pointer for MoveGroupInterface for ease of use in this example
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

void topic_callback(const geometry_msgs::msg::Pose msg)
{
  if (!move_group_interface) {
    RCLCPP_ERROR(rclcpp::get_logger("hello_moveit"), "MoveGroupInterface is not initialized.");
    return;
  }

  move_group_interface->setPoseTarget(msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface->plan(plan));

  if (success) {
    // move_group_interface->execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hello_moveit"), "Planning failed!");
  }
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    // "lbr",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  rclcpp::Logger logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  RCLCPP_INFO(logger, "Creating MoveGroupInterface");
  const MoveGroupInterface::Options opt = MoveGroupInterface::Options("arm", "robot_description", "lbr");
  RCLCPP_INFO(logger, "The thingy returned by Options is %s", opt.move_group_namespace_.c_str());
  
  // move_group_interface = MoveGroupInterface(node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration(0, 0));
  move_group_interface = std::make_shared<MoveGroupInterface>(node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration(0, 0));

  // RCLCPP_INFO(logger, "The thingy returned by Options is %s", move_group_interface.);
  
  // auto move_group_interface = MoveGroupInterface(node, "arm");

  RCLCPP_INFO(logger, "Created");

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_ = node->create_subscription<geometry_msgs::msg::Pose>(
  "moveit_goal", 10, topic_callback);


  // RCLCPP_INFO(node->get_logger(), "Sleeping for 5 seconds...");
  // rclcpp::sleep_for(std::chrono::seconds(5));
  // RCLCPP_INFO(node->get_logger(), "Awake now!");

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}