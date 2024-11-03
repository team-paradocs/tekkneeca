/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/planner_logic_plugins/motion_compensation.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}

namespace moveit::hybrid_planning
{
  bool MotionCompensation::initialize(const std::shared_ptr<HybridPlanningManager>& hybrid_planning_manager)
  {
    hybrid_planning_manager_ = hybrid_planning_manager;
    return true;
  }

  ReactionResult MotionCompensation::react(const HybridPlanningEvent& event)
  {
    switch (event)
    {
      case HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED:
        // Handle new hybrid planning request
        if (hybrid_planning_manager_->calculateIK()) {
          if (checkMotionXsendAction())
          {
            return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
          } else {
            return ReactionResult(event, "Failed to sendAction", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
          }
        } else {
          return ReactionResult(event, "Failed to calculate IK", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        }
      case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
        // Do nothing since we wait for the global planning action to finish
        return ReactionResult(event, "Do nothing", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      
      case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL:
        // Activate local planner once global solution is available
        // ensure the local planner is not started twice
        // if (!local_planner_started_)
        // {
          // Start local planning
          if (!hybrid_planning_manager_->sendLocalPlannerAction(false))
          {
            return ReactionResult(event, "Failed to sendAction", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
          }
          // local_planner_started_ = true;
        // }
        return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_CANCELED:
        return ReactionResult(event, "Global planner actoin canceled",
                      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_ABORTED:
        // Abort hybrid planning if no global solution is found
        return ReactionResult(event, "Global planner failed to find a solution",
                              moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);

      case HybridPlanningEvent::LOCAL_PLANNING_ACTION_SUCCESSFUL:
        // Finish hybrid planning action successfully because local planning action succeeded
        return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      case HybridPlanningEvent::LOCAL_PLANNING_ACTION_CANCELED:
        return ReactionResult(event, "Local planner actoin canceled",
                      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      case HybridPlanningEvent::LOCAL_PLANNING_ACTION_ABORTED:
        // Local planning failed so abort hybrid planning
        return ReactionResult(event, "Local planner failed to find a solution",
                              moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
      default:
        // Unknown event, abort hybrid planning
        return ReactionResult(event, "Unknown event", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    }
  }

  ReactionResult MotionCompensation::react(const std::string& event)
  {
    return ReactionResult(event, "MotionCompensation plugin cannot handle events given as string.",
                          moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  bool MotionCompensation::checkMotionXsendAction()
  {
    RCLCPP_INFO(LOGGER, "In checkMotionXsendAction function");

    auto currentGoalHandle = hybrid_planning_manager_->getHybridPlanningGoalHandle();

    if (!global_planner_started_)
    {
      // First time
      // Start global planning
      RCLCPP_INFO(LOGGER, "Global Planner not started yet, start Global Planner");
      bool clientActionSuccessfullySent = hybrid_planning_manager_->sendGlobalPlannerAction();
      if (!clientActionSuccessfullySent)
      {
        // report failure
        return false;
      }
      global_planner_started_ = true;
    } 
    else 
    {

      double position_difference = calculatePositionDifference(previous_goal_->pose, currentGoalHandle->pose);
      double orientation_difference = calculateOrientationDifference(previous_goal_->pose, currentGoalHandle->pose);
      std::cout << "Position difference: " << position_difference << ", Orientation difference: " << orientation_difference << std::endl;

      if (position_difference > position_threshold_ || orientation_difference > orientation_threshold_) 
      {
        RCLCPP_INFO(LOGGER, "Large motion detected");
        bool clientActionSuccessfullySent = hybrid_planning_manager_->sendGlobalPlannerAction();
        if (!clientActionSuccessfullySent)
        {
          // report failure
          return false;
        }
      } 
      else 
      {
        RCLCPP_INFO(LOGGER, "Small motion detected");
        bool clientActionSuccessfullySent = hybrid_planning_manager_->sendLocalPlannerAction(true);
        if (!clientActionSuccessfullySent)
        {
          // report failure
          return false;
        }
      }

    }
    previous_goal_ = currentGoalHandle;
    return true;
  }

  double MotionCompensation::calculatePositionDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x, 2) + 
                pow(pose1.position.y - pose2.position.y, 2) + 
                pow(pose1.position.z - pose2.position.z, 2));
  }

  double MotionCompensation::calculateOrientationDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) 
  {

    tf2::Quaternion q1(
        pose1.orientation.x,
        pose1.orientation.y,
        pose1.orientation.z,
        pose1.orientation.w
    );

    tf2::Quaternion q2(
        pose2.orientation.x,
        pose2.orientation.y,
        pose2.orientation.z,
        pose2.orientation.w
    );

    // Calculate the relative quaternion: q_relative = q2 * q1.inverse()
    tf2::Quaternion q_relative = q2 * q1.inverse();

    // Normalize the quaternion
    q_relative.normalize();

    // Extract the angle of rotation in radians
    double angle = q_relative.getAngle();

    // Convert to degrees
    double angle_degrees = angle * (180.0 / M_PI);

    return angle_degrees;

  }

  // geometry_msgs::msg::Pose MotionCompensation::constraintsToPose(const moveit_msgs::msg::Constraints& constraints)
  // {

  //   geometry_msgs::msg::Pose pose;
  //   // Assuming constraints.position_constraints has at least one element
  //   if (!constraints.position_constraints.empty()) {
  //       const auto& position_constraint = constraints.position_constraints[0];
  //       pose.position.x = position_constraint.constraint_region.primitive_poses[0].position.x;
  //       pose.position.y = position_constraint.constraint_region.primitive_poses[0].position.y;
  //       pose.position.z = position_constraint.constraint_region.primitive_poses[0].position.z;
  //   }

  //   // Assuming constraints.orientation_constraints has at least one element
  //   if (!constraints.orientation_constraints.empty()) {
  //       const auto& orientation_constraint = constraints.orientation_constraints[0];
  //       pose.orientation.x = orientation_constraint.orientation.x;
  //       pose.orientation.y = orientation_constraint.orientation.y;
  //       pose.orientation.z = orientation_constraint.orientation.z;
  //       pose.orientation.w = orientation_constraint.orientation.w;
  //   }

  //   return pose;
  // }


}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::MotionCompensation, moveit::hybrid_planning::PlannerLogicInterface)
