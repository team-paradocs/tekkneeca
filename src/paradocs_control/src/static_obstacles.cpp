#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/moveit_commander.h>
#include <Eigen/Geometry>
#include "std_msgs/msg/string.hpp"
#include <string>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// rclcpp::Logger* logger;

// moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declare a global pointer for MoveGroupInterface for ease of use in this example
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


geometry_msgs::msg::Pose computePose (const geometry_msgs::msg::Pose pose, float length = 0.02)
{
  //calculate the final drill position
  // Convert the quaternion to a rotation matrix
  Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

  // Create a vector representing the direction to move in
  Eigen::Vector3d direction(0, 0, length);

  // Multiply the rotation matrix by the direction vector
  Eigen::Vector3d result = rotation_matrix * direction;

  // Add the result to the original position to get the new position
  geometry_msgs::msg::Pose new_pose;
  new_pose.position.x = pose.position.x + result(0) - 0.003;
  new_pose.position.y = pose.position.y + result(1) + 0.008; 
  new_pose.position.z = pose.position.z + result(2);

  // The orientation remains the same
  new_pose.orientation = pose.orientation;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << new_pose.position.x << ", " << new_pose.position.y << ", " << new_pose.position.z << "] "
                << "Orientation: [" << new_pose.orientation.x << ", " << new_pose.orientation.y << ", " << new_pose.orientation.z << ", " << new_pose.orientation.w << "]");
  return new_pose;

}
// cartesian drill function
int drillProcess(const geometry_msgs::msg::Pose msg){

  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "in drill function");
  move_group_interface->setStartStateToCurrentState();
  // const geometry_msgs::msg::Pose pose = msg;
  geometry_msgs::msg::Pose pose = computePose(msg, -0.001);
  // print the received message as a string

  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
                << "Orientation: [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]");
  
  // cartesian planning parameters
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);

  int drill_points = 30;

  geometry_msgs::msg::Pose new_pose = computePose(msg, 0.020);
  // Interpolate drill_points number of points between start and end pose
  std::vector<geometry_msgs::msg::Pose> interpolated_poses;
  for (int i = 0; i <= drill_points; i++) {
    geometry_msgs::msg::Pose interpolated_pose;
    float ratio = static_cast<float>(i) / drill_points;

    // Interpolate position
    interpolated_pose.position.x = pose.position.x + ratio * (new_pose.position.x - pose.position.x);
    interpolated_pose.position.y = pose.position.y + ratio * (new_pose.position.y - pose.position.y);
    interpolated_pose.position.z = pose.position.z + ratio * (new_pose.position.z - pose.position.z);

    // Orientation remains the same
    interpolated_pose.orientation = pose.orientation;

    interpolated_poses.push_back(interpolated_pose);
    waypoints.push_back(interpolated_pose);
  }

    // auto message = std_msgs::msg::String();
    // message.data = "d" ;
    // publisher_->publish(message);


  //drill in 
  for (int i=0; i<drill_points; i++){
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface->execute(trajectory);

  }

  //drill out 
  for (int i=drill_points; i>0; i--){
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); 
    move_group_interface->execute(trajectory);
  }

    // RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Drill done");
    // message.data = "s" ;
    // publisher_->publish(message);

  return 0;

}



// cartesian drill function
int cartesianMotion(const geometry_msgs::msg::Pose intermediatePose, const geometry_msgs::msg::Pose goalPose, int cartesian_points = 50){

  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "in cartesian motion function");
  move_group_interface->setStartStateToCurrentState();
  // print the received message as a string

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
  //               << "Orientation: [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]");
  
  // cartesian planning parameters
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  // waypoints.push_back(pose);




  // int cartesian_points = 50;
  // change thhe num,berr of points to interpolate



  // geometry_msgs::msg::Pose new_pose = computePose(pose, 0.02);
  // Interpolate drill_points number of points between start and end pose
  std::vector<geometry_msgs::msg::Pose> interpolated_poses;
  for (int i = 0; i <= cartesian_points; i++) {
    geometry_msgs::msg::Pose interpolated_pose;
    float ratio = static_cast<float>(i) / cartesian_points;

    // Interpolate position
    interpolated_pose.position.x = intermediatePose.position.x + ratio * (goalPose.position.x - intermediatePose.position.x);
    interpolated_pose.position.y = intermediatePose.position.y + ratio * (goalPose.position.y - intermediatePose.position.y);
    interpolated_pose.position.z = intermediatePose.position.z + ratio * (goalPose.position.z - intermediatePose.position.z);

    // Orientation remains the same
    interpolated_pose.orientation = intermediatePose.orientation;

    interpolated_poses.push_back(interpolated_pose);
    // waypoints.push_back(interpolated_pose);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"),"Position: [" << interpolated_pose.position.x << ", " << interpolated_pose.position.y << ", " << interpolated_pose.position.z << "] "
           << "Orientation: [" << interpolated_pose.orientation.x << ", " << interpolated_pose.orientation.y << ", " << interpolated_pose.orientation.z << ", " << interpolated_pose.orientation.w << "]");
  }


  //follow interpolated path 
  for (int i=0; i<cartesian_points; i++){
    RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "cartesian motion");
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface->execute(trajectory);

  }


  return 0;

}


void goToGoal(const geometry_msgs::msg::Pose msg)
{
  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "msg recd");
  if (!move_group_interface) {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "MoveGroupInterface is not initialized.");
    return;
  }
  //cartesian
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // moveit_msgs::msg::RobotTrajectory trajectory;
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // waypoints.push_back(msg);
  // double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);  
  // move_group_interface->execute(trajectory);
  // joint space planning
  // move_group_interface->setStartStateToCurrentState();


  geometry_msgs::msg::Pose new_pose = computePose(msg, -0.07);

  geometry_msgs::msg::PoseStamped gt_pose;
  
  gt_pose.header.frame_id = "world";
  gt_pose.header.stamp = node->get_clock()->now();
  gt_pose.pose = new_pose;
  geometry_msgs::msg::PoseStamped intermediate_pose=gt_pose;
  // intermediate_pose.pose.orientation.x = 0.0;
  // intermediate_pose.pose.orientation.y = 1.0;
  // intermediate_pose.pose.orientation.z = 0.0;
  // intermediate_pose.pose.orientation.w = 0.0;
  move_group_interface->setPoseTarget(intermediate_pose);
  move_group_interface->setPlanningTime(10.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan_intermediate;
  bool success_intermediate = static_cast<bool>(move_group_interface->plan(plan_intermediate));
  if (success_intermediate) {
    // draw_trajectory_tool_path(plan.trajectory);
    move_group_interface->execute(plan_intermediate);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "Planning failed!");
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  move_group_interface->setStartStateToCurrentState();
  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "doneeeeeeeeeeeeeeeee");
  //cartesianMotion(new_pose, msg);

  //intermediate position 1.5cm away
  geometry_msgs::msg::Pose final_pose = computePose(msg, -0.015);
  //new pose is the intermediate position 5cm away
  cartesianMotion(new_pose, final_pose, 70);


  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "starting drill motiuon");
  auto message = std_msgs::msg::String();
  message.data = "d" ;
  publisher_->publish(message);

  //touch bone pose is the position 1mm away
  geometry_msgs::msg::Pose touch_bone_pose = computePose(msg, -0.001);
  //final_pose is the position 1.5cm away
  cartesianMotion(final_pose, touch_bone_pose, 15);
  rclcpp::sleep_for(std::chrono::seconds(1));


  drillProcess(msg);

  rclcpp::sleep_for(std::chrono::seconds(1));

  //go from the touch_bone_pose to the pose that is 1.5cm away
  cartesianMotion(touch_bone_pose, final_pose, 15);

  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Drill done");
  message.data = "s" ;
  publisher_->publish(message);

  // gt_pose.header.frame_id = "world";
  // gt_pose.header.stamp = node->get_clock()->now();
  // move_group_interface->setPoseTarget(gt_pose);
  // move_group_interface->setPlanningTime(10.0);

  
  // float joints[] = {0.0, -30.0,0.5,0.0,0.0,-30,0.0};
  // std::vector<double> joint_group_positions_arm;
  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  // joint_group_positions_arm[1] = -30.0;  // Shoulder Lift
  // joint_group_positions_arm[2] = 0.0;  // Elbow
  // joint_group_positions_arm[3] = 0.0;  // Wrist 1
  // joint_group_positions_arm[4] = 0.0;  // Wrist 2
  // joint_group_positions_arm[5] = -30.0;  // Wrist 3
  // joint_group_positions_arm[6] = 0.0; 

  // move_group_interface->setJointValueTarget(msg, "link_tool");
  // move_group_interface->setMaxVelocityScalingFactor	(	0.1)	;

  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // bool success = static_cast<bool>(move_group_interface->plan(plan));

  // if (success) {
  //   // draw_trajectory_tool_path(plan.trajectory);
  //   move_group_interface->execute(plan);
  //   RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Plan was finally executed. Start Drill");
  //   auto message = std_msgs::msg::String();
  //   message.data = "drill" ;
  //   publisher_->publish(message);
  //   // Uncomment the following line to drill
  //   // drill(gt_pose);
  //   RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Drill done");
  //   message.data = "stop" ;
  //   publisher_->publish(message);
    
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "Planning failed!");
  // }
}

int main(int argc, char * argv[])
{
  
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
    "static_obstacles",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );


  // Create a ROS logger
  rclcpp::Logger logger = rclcpp::get_logger("static_obstacles");
  

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  RCLCPP_INFO(logger, "Creating MoveGroupInterface");
  const MoveGroupInterface::Options opt = MoveGroupInterface::Options("arm", "robot_description", "lbr");
  // RCLCPP_INFO(logger, "The thingy returned by Options is %s", opt.move_group_namespace_.c_str());
  move_group_interface = std::make_shared<MoveGroupInterface>(node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration(0, 0));
  RCLCPP_INFO(logger, "Created");


  // publisher to talk to the drill node
  publisher_ = node->create_publisher<std_msgs::msg::String>("drill_commands", 10);


  //define a planning scene interface and get information about the robot
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());
  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(move_group_interface->getJointModelGroupNames().begin(), move_group_interface->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Set the planner
  std::string planner_plugin_name = "ompl_interface/OMPLPlanner";
  move_group_interface->setPlannerId("RRTstarkConfigDefault");

  // move_group_interface->setPlannerId("LazyPRMstarkConfigDefault");
  // move_group_interface->setPlannerId("SPARStwokConfigDefault"); // works, but not the best
  // move_group_interface->setPlannerId("TRRTkConfigDefault"); // unrealiable, but gives beautiful paths
  // move_group_interface->setPlannerId("LBTRRTkConfigDefault"); // mostly no

  


  // std::string frame_id = "world";
  // // add obstacles to the scene
  // moveit_msgs::msg::CollisionObject collision_object;
  // collision_object.header.frame_id = frame_id;

  // collision_object.id = "box1";
  // shape_msgs::msg::SolidPrimitive primitive;
  // // Define the size of the box in meters
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.2;
  // primitive.dimensions[primitive.BOX_Y] = 0.1;
  // primitive.dimensions[primitive.BOX_Z] = 0.7;
  // geometry_msgs::msg::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0;
  // box_pose.position.y = 0.3;
  // box_pose.position.z = 0.25;
  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // collision_object.id = "box2";
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.05;
  // primitive.dimensions[primitive.BOX_Y] = 1.1;
  // primitive.dimensions[primitive.BOX_Z] = 1.5;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.25;
  // box_pose.position.y = 0;
  // box_pose.position.z = 0.75;
  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // collision_object.id = "table";
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 1.1;
  // primitive.dimensions[primitive.BOX_Y] = 1.1;
  // primitive.dimensions[primitive.BOX_Z] = 0.02;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = -0.25;
  // box_pose.position.y = 0;
  // box_pose.position.z = -0.02;
  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;
  // planning_scene_interface.applyCollisionObject(collision_object);


  // Construct and initialize MoveItVisualTools
  // auto moveit_visual_tools =
  //     moveit_visual_tools::MoveItVisualTools{ node, "link_0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
  //                                             move_group_interface.getRobotModel() };
  // moveit_visual_tools.deleteAllMarkers();
  // moveit_visual_tools.loadRemoteControl();
  // auto const draw_trajectory_tool_path =
  //   [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm")](
  //       auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_ = node->create_subscription<geometry_msgs::msg::Pose>(
  "moveit_goal", 10, goToGoal);
   
  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}