#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include <ompl/base/Planner.h>
#include <ompl/geometric/PathGeometric.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <cmath>

class PredictablePlanner : public ompl::base::Planner {
public:
    PredictablePlanner(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si) {}

    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override {
        std::vector<double> start = {0.0, 0.0, 0.0};  // Example start position
        geometry_msgs::Pose goal;
        goal.position.x = 1.0;  // Example goal position
        goal.position.y = 1.0;
        goal.position.z = 1.0;

        // Generate a predictable plan using direct interpolation
        moveit::planning_interface::MoveGroupInterface::Plan plan = predictable_plan(start, goal);

        // For demonstration purposes, printing the plan
        if (plan.plan_success) {
            std::cout << "Plan found!" << std::endl;
            // Optionally, print the interpolated waypoints
            for (const auto &state : plan.trajectory.joint_trajectory.points) {
                std::cout << "Waypoint: ";
                for (double pos : state.positions) {
                    std::cout << pos << " ";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "Plan failed." << std::endl;
        }

        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    // The function to interpolate the path and create a predictable plan.
    moveit::planning_interface::MoveGroupInterface::Plan predictable_plan(
        const std::vector<double> &start, const geometry_msgs::Pose &goal) {

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Create a trajectory to hold the interpolated path
        trajectory_msgs::JointTrajectory trajectory;

        // Number of interpolation steps
        size_t num_steps = 100;  // You can change this to control the resolution

        // Interpolate the path from start to goal
        for (size_t i = 0; i < num_steps; ++i) {
            // Calculate interpolation factor (0 to 1)
            double t = static_cast<double>(i) / (num_steps - 1);

            // Interpolate the joint positions linearly
            std::vector<double> interpolated_position = interpolate(start, goal, t);

            // Add the interpolated position to the trajectory
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = interpolated_position;
            trajectory.points.push_back(point);
        }

        // Set the plan's trajectory (for demonstration purposes)
        plan.trajectory_ = trajectory;
        plan.plan_success = true;

        return plan;
    }

private:
    // Simple linear interpolation between two states
    std::vector<double> interpolate(const std::vector<double> &start, const geometry_msgs::Pose &goal, double t) {
        std::vector<double> interpolated_state;
        
        // Interpolate each joint angle (if it's a 3D pose)
        interpolated_state.push_back(start[0] + t * (goal.position.x - start[0]));
        interpolated_state.push_back(start[1] + t * (goal.position.y - start[1]));
        interpolated_state.push_back(start[2] + t * (goal.position.z - start[2]));

        return interpolated_state;
    }
};

