#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <string>
#include "rclcpp/wait_for_message.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <Eigen/Geometry>
// Global pointers and variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
geometry_msgs::msg::Pose home_pose;
// Offsets fom SVD (world frame) (For D405)
// float x_offset = 0.002;
// float y_offset = 0.008;
float x_offset = 0.00;
float y_offset = 0.00;
int plan_flag = 0;

int testing_mode = 0;

geometry_msgs::msg::Pose computePose(const geometry_msgs::msg::Pose pose, float length = 0.02)
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
  new_pose.position.x = pose.position.x + result(0) - x_offset;
  new_pose.position.y = pose.position.y + result(1) + y_offset;
  new_pose.position.z = pose.position.z + result(2);

  // The orientation remains the same
  new_pose.orientation = pose.orientation;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "Position: [" << new_pose.position.x << ", " << new_pose.position.y << ", " << new_pose.position.z << "] "
    << "Orientation: [" << new_pose.orientation.x << ", " << new_pose.orientation.y << ", " << new_pose.orientation.z << ", " << new_pose.orientation.w << "]");
  return new_pose;

}

int drillProcess(const geometry_msgs::msg::Pose msg) {

  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "in drill function");
  move_group_interface->setStartStateToCurrentState();
  geometry_msgs::msg::Pose pose = computePose(msg, -0.001);

  // print the received message as a string
  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "Position: [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
    << "Orientation: [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]");

  // cartesian planning parameters
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pose);

  int drill_points = 25;

  geometry_msgs::msg::Pose new_pose = computePose(msg, 0.015);
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


  //drill in 
  for (int i = 0; i < drill_points; i++) {
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface->execute(trajectory);

  }

  //drill out 
  for (int i = drill_points; i > 0; i--) {
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface->execute(trajectory);
  }

  return 0;

}

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::msg::Pose pose) {
  return Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

int cartesianMotion(const geometry_msgs::msg::Pose intermediatePose, const geometry_msgs::msg::Pose goalPose, int cartesian_points = 50) {

  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "in cartesian motion function");
  move_group_interface->setStartStateToCurrentState();

  // cartesian planning parameters
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;



  // Eigen::Quaterniond q1 = toEigenQuaternion(intermediatePose);
  // Eigen::Quaterniond q2 = toEigenQuaternion(goalPose);



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
    // interpolated_pose.position = intermediatePose.position;


    // double t = i / static_cast<double>(cartesian_points);  // Calculate the interpolation parameter
    // Eigen::Quaterniond qi = q1.slerp(t, q2);

    // Orientation remains the same
    interpolated_pose.orientation = intermediatePose.orientation;
    // interpolated_pose.orientation.x = qi.x();
    // interpolated_pose.orientation.y = qi.y();
    // interpolated_pose.orientation.z = qi.z();
    // interpolated_pose.orientation.w = qi.w();

    interpolated_poses.push_back(interpolated_pose);
    // waypoints.push_back(interpolated_pose);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "Position: [" << interpolated_pose.position.x << ", " << interpolated_pose.position.y << ", " << interpolated_pose.position.z << "] "
      << "Orientation: [" << interpolated_pose.orientation.x << ", " << interpolated_pose.orientation.y << ", " << interpolated_pose.orientation.z << ", " << interpolated_pose.orientation.w << "]");
  }


  //follow interpolated path 
  for (int i = 0; i < cartesian_points; i++) {
    RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "cartesian motion");
    waypoints.clear();
    waypoints.push_back(interpolated_poses[i]);
    move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface->execute(trajectory);

  }


  return 0;

}

int jointSpaceMotion(const geometry_msgs::msg::Pose goalPose)
{
  geometry_msgs::msg::PoseStamped gt_pose;
  gt_pose.header.frame_id = "world";
  gt_pose.header.stamp = node->get_clock()->now();
  gt_pose.pose = goalPose;
  geometry_msgs::msg::PoseStamped intermediate_pose = gt_pose;
  move_group_interface->setPoseTarget(intermediate_pose);
  // move_group_interface->setPlanningTime(10.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan_intermediate;
  bool success_intermediate = static_cast<bool>(move_group_interface->plan(plan_intermediate));


  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "please confirm the plan by setting the plan_flag to 1");
  std_msgs::msg::Int32 inpmsg;
  bool recd = rclcpp::wait_for_message(inpmsg, node, "plan_flag", std::chrono::seconds(20));
  if (recd)
  {
    plan_flag = inpmsg.data;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "plan_flag not recd");
    return jointSpaceMotion(goalPose);
  }


  RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "plan_flag is " << plan_flag << ",");
  if (plan_flag == 1) {
    plan_flag = 0;
    if (success_intermediate) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "executing " << plan_flag << ",");
      // draw_trajectory_tool_path(plan.trajectory);
      move_group_interface->execute(plan_intermediate);
    }
    else {
      RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "Planning failed!");
      return jointSpaceMotion(goalPose);
    }
    return 0;
  }
  else if (plan_flag == 2) {
    return 1;
  }
  else {
    return jointSpaceMotion(goalPose);
  }




  return 0;
}

void goToGoalCallback(const geometry_msgs::msg::PoseArray msg_array)
{
  RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "msg recd");
  if (!move_group_interface) {
    RCLCPP_ERROR(rclcpp::get_logger("static_obstacles"), "MoveGroupInterface is not initialized.");
    return;
  }

  // // Compute an intermediate pose at a safe distance from the target
  // geometry_msgs::msg::Pose intermediate_pose_7cm = computePose(msg_array.poses[0], -0.07);

  // geometry_msgs::msg::Pose intermediate_pose_7cm_translation = intermediate_pose_7cm;
  // intermediate_pose_7cm_translation.orientation.x = 0.0;
  // intermediate_pose_7cm_translation.orientation.y = 1.0;
  // intermediate_pose_7cm_translation.orientation.z = 0.0;
  // intermediate_pose_7cm_translation.orientation.w = 0.0;
  // // go to the intermediate pose from home
  // int is_failure = jointSpaceMotion(intermediate_pose_7cm_translation);
  // move_group_interface->setStartStateToCurrentState();


  // int is_failure = jointSpaceMotion(intermediate_pose_7cm);
  // move_group_interface->setStartStateToCurrentState();

  /////////////////////////////working

  // geometry_msgs::msg::Pose currentPose = intermediate_pose_7cm_translation;

  // if (!is_failure) {
    // if (false) {
      // go to an intermediate position 1.5cm away
  for (size_t i = 0; i < msg_array.poses.size(); i++) {
    geometry_msgs::msg::Pose msg = msg_array.poses[i];
    rclcpp::sleep_for(std::chrono::seconds(1));
    move_group_interface->setStartStateToCurrentState();
    geometry_msgs::msg::Pose intermediate_pose_7cm = computePose(msg, -0.07);
    int is_failure = jointSpaceMotion(intermediate_pose_7cm);
    move_group_interface->setStartStateToCurrentState();

    if (!is_failure) {


      rclcpp::sleep_for(std::chrono::seconds(1));
      geometry_msgs::msg::Pose intermediate_pose_15mm = computePose(msg, -0.015);
      cartesianMotion(intermediate_pose_7cm, intermediate_pose_15mm, 70);

      // switch on the drill
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "starting drill motion");
      auto message = std_msgs::msg::String();
      message.data = "d";
      publisher_->publish(message);


      if (!testing_mode) {
        // touch the bone
        geometry_msgs::msg::Pose touch_bone_pose = computePose(msg, -0.001);
        cartesianMotion(intermediate_pose_15mm, touch_bone_pose, 15);
        rclcpp::sleep_for(std::chrono::seconds(1));

        // drill the bone
        drillProcess(msg);

        // go from the touch_bone_pose to the pose that is 1.5cm away
        rclcpp::sleep_for(std::chrono::seconds(1));
        cartesianMotion(touch_bone_pose, intermediate_pose_15mm, 15);

      }

      else {
        rclcpp::sleep_for(std::chrono::seconds(5));
      }

      // switch off the drill
      RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Drill done");
      message.data = "s";
      publisher_->publish(message);

      // go back to the intermediate pose 7cm away
      rclcpp::sleep_for(std::chrono::seconds(1));
      cartesianMotion(intermediate_pose_15mm, intermediate_pose_7cm, 70);
      // currentPose = intermediate_pose_7cm;

      // // go back home
      // rclcpp::sleep_for(std::chrono::seconds(1));
      // jointSpaceMotion(home_pose);
    }
  }
}
// }


void update_plan_flag(const std_msgs::msg::Int32::SharedPtr msg) {
  plan_flag = msg->data;
}

int main(int argc, char* argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>(
    "static_obstacles",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  home_pose.position.x = -0.4;
  home_pose.position.y = 0.0;
  home_pose.position.z = 0.365;
  home_pose.orientation.x = 0.0;
  home_pose.orientation.y = 1.0;
  home_pose.orientation.z = 0.0;
  home_pose.orientation.w = 0.0;


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

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "moveit_goal", 10, goToGoalCallback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_ = node->create_subscription<std_msgs::msg::Int32>(
    "plan_flag", 10, update_plan_flag);

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}