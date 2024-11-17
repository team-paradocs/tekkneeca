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

      case HybridPlanningEvent::DRILLING_REQUEST_RECEIVED:
        // Handle new drilling request
        // Do nothing since we wait for the drilling action to finish
        // the state is drill_state_ is 1
        hybrid_planning_manager_->drillMotion();
        return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
        // this event can be deleted
        // Do nothing since we wait for the global planning action to finish
        return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL:
        // Start local planning
        if (!hybrid_planning_manager_->sendLocalPlannerAction(1))
        {
          return ReactionResult(event, "Failed to sendAction", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        }
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
        if (hybrid_planning_manager_->getDrillState() == 0)
        {
          // does nothing
        }
        else if (hybrid_planning_manager_->getDrillState() == 1)
        {
          // arrive at the predrill pose
          hybrid_planning_manager_->setDrillState(2);
          hybrid_planning_manager_->drillMotion();
        }
        else if (hybrid_planning_manager_->getDrillState() == 2)
        {
          // arrive at the startdrill pose
          hybrid_planning_manager_->setDrillState(3);
          hybrid_planning_manager_->drillCmd(true);
          hybrid_planning_manager_->drillMotion();
        }
        else if (hybrid_planning_manager_->getDrillState() == 3)
        {
          hybrid_planning_manager_->setDrillState(4);
          hybrid_planning_manager_->drillMotion();
        }
        else if (hybrid_planning_manager_->getDrillState() == 4)
        {
          hybrid_planning_manager_->setDrillState(5);
          // backed to predrill
          hybrid_planning_manager_->drillCmd(false);
          hybrid_planning_manager_->drillMotion();
        }
        else if (hybrid_planning_manager_->getDrillState() == 5)
        {
          // homed
          hybrid_planning_manager_->setDrillState(0);
          hybrid_planning_manager_->drillMotion();
        }
        else if (hybrid_planning_manager_->getDrillState() == 6)
        {
          // homed
          hybrid_planning_manager_->setDrillState(0);
        }
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
    // react to eveents that are not defined in the HybridPlanningEvent enum
    // return ReactionResult(event, "MotionCompensation plugin cannot handle events given as string.",
    //                       moveit_msgs::msg::MoveItErrorCodes::FAILURE);

    if ((event == toString(LocalFeedbackEnum::COLLISION_AHEAD)) ||
        (event == toString(LocalFeedbackEnum::LOCAL_PLANNER_STUCK)))
    {

      // stop executing the local plan
      hybrid_planning_manager_->stopLocalPlanner();

      bool clientActionSuccessfullySent = hybrid_planning_manager_->sendGlobalPlannerAction(false);
      if (!clientActionSuccessfullySent)
      {
        // report failure
        return ReactionResult(event, "Failed to sendGlobalPlannerAction", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      }
      // start local
      clientActionSuccessfullySent = hybrid_planning_manager_->sendLocalPlannerAction(1);
      if (!clientActionSuccessfullySent)
      {
        // report failure
        return ReactionResult(event, "Failed to sendLocalPlannerAction", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      }
      return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    }
    else
    {
      return ReactionResult(event, "MotionCompensation plugin cannot handle this event.",
                            moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    }

  }

  void MotionCompensation::reset()
  {
    first_time_ = true;
    previous_goal_ = nullptr;
  }

  bool MotionCompensation::checkMotionXsendAction()
  {
    // RCLCPP_INFO(LOGGER, "In checkMotionXsendAction function");

    auto currentGoalHandle = hybrid_planning_manager_->getHybridPlanningGoalHandle();

    if (first_time_)
    {
      // First time
      // hybrid_planning_manager_->stopLocalPlanner();

      // Start global planning
      // RCLCPP_INFO(LOGGER, "Global Planner not started yet, start Global Planner");
      bool clientActionSuccessfullySent = hybrid_planning_manager_->sendGlobalPlannerAction(false);
      if (!clientActionSuccessfullySent)
      {
        // report failure
        return false;
      }

      // Start local planning (already started the execution loop in initialize (or through exe sub))
      // RCLCPP_INFO(LOGGER, "Local Planner not started yet, start Local Planner");
      // clientActionSuccessfullySent = hybrid_planning_manager_->sendLocalPlannerAction(1);
      // if (!clientActionSuccessfullySent)
      // {
      //   // report failure
      //   return false;
      // }

      first_time_ = false;
    } 
    else 
    {

      double position_difference = calculatePositionDifference(previous_goal_->pose, currentGoalHandle->pose);
      double orientation_difference = calculateOrientationDifference(previous_goal_->pose, currentGoalHandle->pose);
      // std::cout << "Position difference: " << position_difference << ", Orientation difference: " << orientation_difference << std::endl;

      // if (position_difference < compensation_position_threshold_) 
      // {
      //   // RCLCPP_INFO(LOGGER, "Subtle motion detected, not compensating");
      //   return true;
      // }

      if (position_difference > position_threshold_ || orientation_difference > orientation_threshold_) 
      {
        RCLCPP_INFO(LOGGER, "Large motion detected");
        
        // stop executing the local plan
        hybrid_planning_manager_->stopLocalPlanner();

        bool clientActionSuccessfullySent = hybrid_planning_manager_->sendGlobalPlannerAction(false);
        if (!clientActionSuccessfullySent)
        {
          // report failure
          return false;
        }
        // await global
        clientActionSuccessfullySent = hybrid_planning_manager_->sendLocalPlannerAction(1);
        if (!clientActionSuccessfullySent)
        {
          // report failure
          return false;
        }
      } 
      else 
      {
        RCLCPP_INFO(LOGGER, "Small motion detected");
        bool clientActionSuccessfullySent = hybrid_planning_manager_->sendLocalPlannerAction(2);
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

}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::MotionCompensation, moveit::hybrid_planning::PlannerLogicInterface)
