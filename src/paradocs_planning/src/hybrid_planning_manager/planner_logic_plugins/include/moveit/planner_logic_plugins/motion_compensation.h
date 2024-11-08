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

/* Author: Sebastian Jahr
   Description: This planner logic plugin runs the global planner once and starts executing the global solution
    with the local planner.
 */

#include <moveit/hybrid_planning_manager/planner_logic_interface.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>
#include <moveit/local_planner/feedback_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace moveit::hybrid_planning
{
class MotionCompensation : public PlannerLogicInterface
{
public:
  MotionCompensation() = default;
  ~MotionCompensation() override = default;
  bool initialize(const std::shared_ptr<HybridPlanningManager>& hybrid_planning_manager) override;
  ReactionResult react(const HybridPlanningEvent& event) override;
  ReactionResult react(const std::string& event) override;
  void reset();

private:
  bool checkMotionXsendAction();
  double calculateOrientationDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2); 
  double calculatePositionDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

  // threshold for the position difference (m)
  double position_threshold_ = 0.07;

  // threshold for the orientation difference (deg)
  double orientation_threshold_ = 90.0;

  // Previous goal
  std::shared_ptr<const geometry_msgs::msg::PoseStamped> previous_goal_ = nullptr;

  // bool local_planner_started_ = false;
  bool first_time_ = true;
};
}  // namespace moveit::hybrid_planning
