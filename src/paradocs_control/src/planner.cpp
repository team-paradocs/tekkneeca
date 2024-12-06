#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>

int number_of_waypoints = 10;
int intermediate_steps = 5;
const double step_size = 0.05;
const int max_iterations = 5000;
const double goal_threshold = 0.02;
double random_threshold = 0.2;
double extend_epsilon = 5;
double valid_cartesian_limit = 0.1;

const int dimensions = 7;
double PI = 3.14159265358979323846;

class PredictablePlanner {
public:

    PredictablePlanner(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {

        moveit_cpp::MoveItCpp::Options moveit_cpp_options(node_);
        moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_, moveit_cpp_options);
        planning_scene_monitor_ = moveit_cpp_->getPlanningSceneMonitor();

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        if (!planning_scene_monitor_->getPlanningScene()) {
            throw std::runtime_error("Failed to initialize PlanningSceneMonitor!");
        }

        robot_model_ = moveit_cpp_->getRobotModel();
        goal_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(
            goal_state_->getJointModelGroup("arm"));

        std::cout << "PredictablePlanner initialized" << std::endl;
    }

    bool is_valid(const std::vector<double>& joint_values, const std::vector<std::vector<double>>& waypoints) {
        try {
            if (joint_values.size() != 7) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid number of joint values");
                return false;
            }
            const std::string group_name = "arm";

            // Get the current robot state
            auto planning_scene = planning_scene_monitor_->getPlanningScene();
            moveit::core::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

            // Set joint group positions
            robot_state.setJointGroupPositions(group_name, joint_values);
            robot_state.update();  // Update the state to reflect the new joint values

            // Check state validity
            bool is_valid = planning_scene->isStateValid(robot_state, group_name);
            if (is_valid) {
                // RCLCPP_INFO(node_->get_logger(), "The state is valid!");
            }
            else {
                // RCLCPP_INFO(node_->get_logger(), "The state is invalid!");
                return false;
            }

            // Check for collisions
            bool is_collision_free = !planning_scene->isStateColliding(robot_state, group_name);
            if (is_collision_free) {
                // RCLCPP_INFO(node_->get_logger(), "The state is collision-free!");
                return true;
            }
            else {
                // RCLCPP_INFO(node_->get_logger(), "The state is in collision!");
                return false;
            }

            // Check if the distance of the point to the closest waypoint is greater than a threshold
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& waypoint : waypoints) {
                double dist = distance(joint_values, waypoint);
                if (dist < min_distance) {
                    min_distance = dist;
                }
            }

            if (min_distance > valid_cartesian_limit) {
                RCLCPP_ERROR(node_->get_logger(), "The point is too far from the closest waypoint");
                return false;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in is_valid: %s", e.what());
            return false;
        }

    }

    std::vector<geometry_msgs::msg::Pose> cartesian_interpolate(const geometry_msgs::msg::Pose intermediatePose, const geometry_msgs::msg::Pose goalPose)
    {
        int cartesian_points = number_of_waypoints;
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
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("static_obstacles"), "Position: [" << interpolated_pose.position.x << ", " << interpolated_pose.position.y << ", " << interpolated_pose.position.z << "] "
            //     << "Orientation: [" << interpolated_pose.orientation.x << ", " << interpolated_pose.orientation.y << ", " << interpolated_pose.orientation.z << ", " << interpolated_pose.orientation.w << "]");
        }
        return interpolated_poses;
    }

    std::vector<std::vector<double>> waypoints_ik(std::vector<geometry_msgs::msg::Pose> waypoints)
    {
        std::vector<std::vector<double>> joint_values_array;
        for (const auto& waypoint : waypoints) {
            bool ik_success;
            // RCLCPP_INFO(node_->get_logger(), "Waypoint position - x: %f, y: %f, z: %f", waypoint.position.x, waypoint.position.y, waypoint.position.z);
            // RCLCPP_INFO(node_->get_logger(), "Waypoint orientation - x: %f, y: %f, z: %f, w: %f", waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w);
            ik_success = goal_state_->setFromIK(joint_model_group_.get(), waypoint);
            if (ik_success) {
                std::vector<double> joint_values;
                goal_state_->copyJointGroupPositions(joint_model_group_.get(), joint_values);
                joint_values_array.push_back(joint_values);
            }
            else {
                RCLCPP_ERROR(node_->get_logger(), "IK failed for waypoint");
            }
        }
        return joint_values_array;
    }

    moveit_msgs::msg::RobotTrajectory interpolate_points(const std::vector<std::vector<double>> waypoints, const std::vector<double> start) {

        moveit_msgs::msg::RobotTrajectory trajectory;
        trajectory.joint_trajectory.joint_names = robot_model_->getJointModelGroup("arm")->getVariableNames();

        int num_waypoints = number_of_waypoints;

        for (int i = 1; i < num_waypoints; i++) {
            std::vector<double> waypoint = waypoints[i];

            RCLCPP_INFO(node_->get_logger(), "Waypoint %d:", i);
            for (size_t j = 0; j < waypoint.size(); ++j) {
                RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", j, waypoint[j]);
            }
            for (int k = 0; k < intermediate_steps; k++) {

                trajectory_msgs::msg::JointTrajectoryPoint point;
                double t = static_cast<double>(k) / (intermediate_steps - 1);  // Interpolation parameter from 0 to 1

                for (size_t j = 0; j < start.size(); ++j) {
                    double interpolated_value = waypoints[i - 1][j] + t * (waypoints[i][j] - waypoints[i - 1][j]);
                    point.positions.push_back(interpolated_value);
                }

                point.time_from_start = rclcpp::Duration::from_seconds((i + t) * 2);  // Set the time for this waypoint
                // trajectory.joint_trajectory.points[i] = point;
                trajectory.joint_trajectory.points.push_back(point);
                RCLCPP_INFO(node_->get_logger(), "Interpolated point at step %d:", k);
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", j, point.positions[j]);
                }
            }

        }
        return trajectory;
    }


    void clear() {
        std::cout << "Clearing planner state..." << std::endl;
    }

    struct Node {
        std::vector<double> position;
        Node* parent;

        Node(const std::vector<double>& pos, Node* par = nullptr) : position(pos), parent(par) {}
    };

    using Tree = std::vector<Node*>;

    std::vector<double> sampleClosePoint(std::vector<std::vector<double>> waypoints, int iter) {
        std::vector<double> reference_point = waypoints[iter % 7];

        std::vector<double> random_point;
        for (int i = 0; i < reference_point.size(); ++i) {
            double random_value = reference_point[i] + random_threshold * (2.0 * (std::rand() / (double)RAND_MAX) - 1.0);
            random_point.push_back(random_value);
        }

        return random_point;

    }

    std::vector<double> steer(const std::vector<double>& from, const std::vector<double>& to, double step_size) {
        std::vector<double> direction(to.size());
        double norm = 0.0;

        for (size_t i = 0; i < to.size(); ++i) {
            direction[i] = to[i] - from[i];
            norm += direction[i] * direction[i];
        }

        norm = std::sqrt(norm);
        for (size_t i = 0; i < to.size(); ++i) {
            direction[i] = from[i] + (direction[i] / norm) * step_size;
        }

        return direction;
    }

    bool connect(Tree& tree, const std::vector<double>& target, double step_size, double goal_threshold, std::vector<std::vector<double>> waypoints) {
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.1");
        Node* nearest = nearestNeighbor(tree, target);
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.2");
        std::vector<double> newPoint = steer(nearest->position, target, step_size);
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.3");
        int epsilon = extend_epsilon;
        while (is_valid(newPoint, waypoints) && epsilon > 0) {
            epsilon--;
            Node* newNode = new Node(newPoint, nearest);
            tree.push_back(newNode);
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.4");

            if (distance(newPoint, target) < goal_threshold) {
                // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.5.True");
                return true;
            }
            // if (newPoint == target) {
            //     RCLCPP_INFO(node_->get_logger(), "debug stmt 6.5.True");
            //     return true;
            // }

            nearest = newNode;
            newPoint = steer(nearest->position, target, step_size);
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 6.45");

        }
        RCLCPP_INFO(node_->get_logger(), "debug stmt 6.5");

        return false;
    }

    double distance(const std::vector<double>& a, const std::vector<double>& b) {
        double dist = 0;
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.2.1");
        for (int i = 0; i < 7; ++i) {
            // dist += pow(a[i] - b[i], 2);
            double diff = a[i] - b[i];
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.2.2");

            // Normalize angle difference to [-pi, pi]
            if (diff > PI) diff -= 2 * PI;
            else if (diff < -PI) diff += 2 * PI;
            dist = diff * diff;
            // if(diff>dist)
            // {
            //     dist = sqrt(diff*diff);
            // }
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.2.25");
        }
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.2.3");
        // Calculate the Cartesian position with joint values a

        const std::string group_name = "arm";

        // Get the current robot state
        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        moveit::core::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

        // Set joint group positions
        robot_state.setJointGroupPositions(group_name, a);
        robot_state.update();  // Update the state to reflect the new joint values

        const Eigen::Isometry3d& end_effector_state = robot_state.getGlobalLinkTransform("link_tool");
        geometry_msgs::msg::Pose pose;
        pose.position.x = end_effector_state.translation().x();
        pose.position.y = end_effector_state.translation().y();
        pose.position.z = end_effector_state.translation().z();
        // RCLCPP_INFO(node_->get_logger(), "Pose A - x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z);
        robot_state.setJointGroupPositions(group_name, b);
        robot_state.update();  // Update the state to reflect the new joint values

        const Eigen::Isometry3d& end_effector_state2 = robot_state.getGlobalLinkTransform("link_tool");
        geometry_msgs::msg::Pose poseB;
        poseB.position.x = end_effector_state2.translation().x();
        poseB.position.y = end_effector_state2.translation().y();
        poseB.position.z = end_effector_state2.translation().z();
        // RCLCPP_INFO(node_->get_logger(), "Pose B - x: %f, y: %f, z: %f", poseB.position.x, poseB.position.y, poseB.position.z);

        double cartesian_dist = sqrt(pow(pose.position.x - poseB.position.x, 2) + pow(pose.position.y - poseB.position.y, 2) + pow(pose.position.z - poseB.position.z, 2));
        // RCLCPP_INFO(node_->get_logger(), "Cartesian distance: %f", cartesian_dist);

        return cartesian_dist;
        // return sqrt(dist);
        // return dist;
    }

    Node* nearestNeighbor(const Tree& tree, const std::vector<double>& point) {
        Node* nearest = nullptr;
        double minDist = std::numeric_limits<double>::max();

        for (Node* node : tree) {
            double dist = 0.0;
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.1");
            // RCLCPP_INFO(node_->get_logger(), "Node position:");
            // for (size_t i = 0; i < node->position.size(); ++i) {
            //     RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", i, node->position[i]);
            // }
            // RCLCPP_INFO(node_->get_logger(), "Point position:");
            // for (size_t i = 0; i < point.size(); ++i) {
            //     RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", i, point[i]);
            // }
            dist = distance(node->position, point);
            // RCLCPP_INFO(node_->get_logger(), "debug stmt 3.2");

            if (dist < minDist) {
                minDist = dist;
                nearest = node;
            }
        }

        return nearest;
    }
    std::vector<std::vector<double>> extractPath(Node* start, Node* goal) {
        std::vector<std::vector<double>> path;
        Node* current = goal;

        while (current) {
            path.push_back(current->position);
            current = current->parent;
        }

        return path;
    }

    std::vector<std::vector<double>> RRTConnect(std::vector<std::vector<double>> waypoints) {
        std::vector<double> start = waypoints[0];
        std::vector<double> goal = waypoints[waypoints.size() - 1];


        std::vector<std::vector<double>> path;

        Tree treeA, treeB;
        treeA.push_back(new Node(start));
        treeB.push_back(new Node(goal));

        bool pathFound = false;
        Node* connectingNodeA = nullptr;
        Node* connectingNodeB = nullptr;

        std::srand(std::time(nullptr));
        // RCLCPP_INFO(node_->get_logger(), "debug stmt 1");
        int iter;
        for (iter = 0; iter < max_iterations; ++iter) {
            RCLCPP_INFO(node_->get_logger(), "iter: %d", iter);
            Tree& primaryTree = (iter % 2 == 0) ? treeA : treeB;
            Tree& secondaryTree = (iter % 2 == 0) ? treeB : treeA;

            // RCLCPP_INFO(node_->get_logger(), "debug stmt 2");
            // Sample a random point
            std::vector<double> randomPoint = sampleClosePoint(waypoints, iter);

            RCLCPP_INFO(node_->get_logger(), "debug stmt 3");

            // Find nearest node in the primary tree and attempt to steer
            Node* nearest = nearestNeighbor(primaryTree, randomPoint);

            RCLCPP_INFO(node_->get_logger(), "debug stmt 4");

            std::vector<double> newPoint = steer(nearest->position, randomPoint, step_size);

            RCLCPP_INFO(node_->get_logger(), "debug stmt 5");

            if (is_valid(newPoint, waypoints)) {
                Node* newNode = new Node(newPoint, nearest);
                primaryTree.push_back(newNode);

                RCLCPP_INFO(node_->get_logger(), "debug stmt 6");

                // Try to connect to the secondary tree
                if (connect(secondaryTree, newPoint, step_size, goal_threshold, waypoints)) {

                    RCLCPP_INFO(node_->get_logger(), "debug stmt 7");

                    connectingNodeA = newNode;
                    connectingNodeB = secondaryTree.back();
                    pathFound = true;
                    break;
                }
                RCLCPP_INFO(node_->get_logger(), "debug stmt 8");
            }
        }

        if (pathFound) {
            RCLCPP_INFO(node_->get_logger(), "Path found!");
            std::cout << "Path found!" << std::endl;
            auto pathFromA = extractPath(treeA.front(), connectingNodeA);
            auto pathFromB = extractPath(treeB.front(), connectingNodeB);
            std::reverse(pathFromB.begin(), pathFromB.end());
            pathFromA.insert(pathFromA.end(), pathFromB.begin(), pathFromB.end());
            if (iter % 2 == 0) std::reverse(pathFromA.begin(), pathFromA.end());
            path = pathFromA;

            // Print path
            for (const auto& point : pathFromA) {
                RCLCPP_WARN(node_->get_logger(), "(%f, %f)", point[0], point[1]);
            }
        }
        else {
            std::cout << "Path not found." << std::endl;
        }

        // Cleanup memory
        for (Node* node : treeA) delete node;
        for (Node* node : treeB) delete node;

        return path;

    }

    moveit::planning_interface::MoveGroupInterface::Plan predictable_plan(
        const std::vector<double> start_state, const geometry_msgs::msg::PoseStamped& goal) {


        RCLCPP_INFO(rclcpp::get_logger("static_obstacles"), "Before planning");


        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        moveit::core::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

        // Get the Cartesian position of the link_tool
        const std::string link_name = "link_tool";
        const Eigen::Isometry3d& link_transform = robot_state.getGlobalLinkTransform(link_name);
        geometry_msgs::msg::Pose link_pose;
        link_pose.position.x = link_transform.translation().x();
        link_pose.position.y = link_transform.translation().y();
        link_pose.position.z = link_transform.translation().z();
        link_pose.orientation = goal.pose.orientation;

        RCLCPP_INFO(node_->get_logger(), "Link tool position - x: %f, y: %f, z: %f", link_pose.position.x, link_pose.position.y, link_pose.position.z);
        double start_time = rclcpp::Clock().now().seconds();
        std::vector<geometry_msgs::msg::Pose> cartesian_waypoints = cartesian_interpolate(link_pose, goal.pose);

        std::vector<std::vector<double>> waypoints = waypoints_ik(cartesian_waypoints);

        std::vector<std::vector<double>> path = RRTConnect(waypoints);

        moveit_msgs::msg::RobotTrajectory robot_trajectory;
        robot_trajectory.joint_trajectory.joint_names = robot_state.getVariableNames();

        // Set the joint names for the trajectory
        // robot_trajectory.joint_trajectory.joint_names = joint_model_group->getVariableNames();

        // Populate the trajectory points
        for (size_t i = 0; i < path.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = path[i];
            point.time_from_start = rclcpp::Duration::from_seconds(1 * i); // Assign time for each point
            RCLCPP_INFO(node_->get_logger(), "path %ld:", i);
            // for (size_t j = 0; j < point.positions.size(); ++j) {
            //     RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", j, point.positions[j]);
            // }
            robot_trajectory.joint_trajectory.points.push_back(point);
            // for (size_t j = 0; j < point.positions.size(); ++j) {
            //     RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", j, point.positions[j]);
            // }
        }

        // // Print the joint space message in the trajectory
        // for (size_t i = 0; i < robot_trajectory.joint_trajectory.points.size(); ++i) {
        //     const auto& point = robot_trajectory.joint_trajectory.points[i];
        //     RCLCPP_WARN(node_->get_logger(), "Trajectory point %ld:", i);
        //     for (size_t j = 0; j < point.positions.size(); ++j) {
        //         RCLCPP_WARN(node_->get_logger(), "Joint %ld: %f", j, point.positions[j]);
        //     }
        // }

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        plan.trajectory_ = robot_trajectory;

        RCLCPP_ERROR(node_->get_logger(), "Generated a predictable plan");
        double end_time = rclcpp::Clock().now().seconds();
        RCLCPP_INFO(node_->get_logger(), "Time taken: %f", end_time - start_time);
        return plan;

        // Print the robot trajectory
        // for (size_t i = 0; i < robot_trajectory.joint_trajectory.points.size(); ++i) {
        //     const auto& point = robot_trajectory.joint_trajectory.points[i];
        //     RCLCPP_INFO(node_->get_logger(), "Trajectory point %ld:", i);
        //     for (size_t j = 0; j < point.positions.size(); ++j) {
        //         RCLCPP_INFO(node_->get_logger(), "Joint %ld: %f", j, point.positions[j]);
        //     }
        // }
        // exit(0);?



        // RCLCPP_INFO(node_->get_logger(), "checkpoint");

        // // // Copy the joint values from robot_state to start
        // const std::string group_name = "arm";
        // const moveit::core::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(group_name);
        // std::vector<double> start;
        // robot_state.copyJointGroupPositions(joint_model_group, start);

        // RCLCPP_INFO(node_->get_logger(), "Start joint values: ");
        // for (const auto& value : start) {
        //     RCLCPP_INFO(node_->get_logger(), "%f ", value);
        // }

        // moveit_msgs::msg::RobotTrajectory trajectory;


        // moveit_msgs::msg::RobotTrajectory trajectory = interpolate_points(waypoints, start);



        // // Print start and goal

        // RCLCPP_INFO(node_->get_logger(), "Start joint values: ");
        // for (const auto& value : start) {
        //     RCLCPP_INFO(node_->get_logger(), "%f ", value);
        // }

        // RCLCPP_INFO(node_->get_logger(), "Goal pose: ");
        // RCLCPP_INFO(node_->get_logger(), "Position - x: %f, y: %f, z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        // RCLCPP_INFO(node_->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);


        // bool ik_success;
        // ik_success = goal_state_->setFromIK(joint_model_group_.get(), goal.pose);
        // RCLCPP_INFO(node_->get_logger(), "IK success: %d", ik_success);
        // std::vector<double> joint_values;
        // goal_state_->copyJointGroupPositions(joint_model_group_.get(), joint_values);
        // for (size_t i = 0; i < joint_values.size(); ++i)
        // {
        //     RCLCPP_WARN(node_->get_logger(), "Joint %ld: %f", i + 1, joint_values[i]);
        // }

        // Check if the start state is valid
        // is_valid(start);

        // moveit::planning_interface::MoveGroupInterface::Plan plan;

        // plan.trajectory_ = robot_trajectory;

        // RCLCPP_ERROR(node_->get_logger(), "Generated a predictable plan");
        // return plan;
    }




private:
    std::shared_ptr<rclcpp::Node> node_;
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

    std::string planning_group_ = "arm";

    std::shared_ptr<const moveit::core::RobotModel> robot_model_;
    std::shared_ptr<moveit::core::RobotState> goal_state_;
    std::shared_ptr<const moveit::core::JointModelGroup> joint_model_group_;



};
