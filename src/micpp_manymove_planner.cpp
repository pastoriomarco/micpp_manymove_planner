// src/micpp_manymove_planner.cpp

#include "micpp_manymove_planner/micpp_manymove_planner.hpp"

#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>

#include <memory>
#include <vector>

// Using declarations for cleaner code
using micpp_manymove_planner::msg::MovementConfig;

// Constructor
ManyMovePlanner::ManyMovePlanner(
    const rclcpp::Node::SharedPtr& node,
    const std::string& planning_group,
    const std::string& base_frame,
    const std::string& tcp_frame)
: node_(node), logger_(node->get_logger()), base_frame_(base_frame), tcp_frame_(tcp_frame)
{
    // Initialize MoveItCpp
    moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);

    // Provide the PlanningScene service
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    // Initialize PlanningComponent
    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group, moveit_cpp_ptr_);
    RCLCPP_INFO(logger_, "ManyMovePlanner initialized with group: %s", planning_group.c_str());
}

// Compute Path Length
double ManyMovePlanner::computePathLength(const robot_trajectory::RobotTrajectory &trajectory) const
{
    double total_length = 0.0;

    // Compute joint-space path length
    auto computeJointPathLength = [&](const robot_trajectory::RobotTrajectory &traj) -> double
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.getWayPointCount(); ++i)
        {
            const auto &prev = traj.getWayPoint(i - 1);
            const auto &curr = traj.getWayPoint(i);
            std::vector<double> prev_positions, curr_positions;
            prev.copyJointGroupPositions(planning_components_->getPlanningGroupName(), prev_positions);
            curr.copyJointGroupPositions(planning_components_->getPlanningGroupName(), curr_positions);
            double seg_len = 0.0;
            for (size_t j = 0; j < prev_positions.size(); ++j)
            {
                double diff = curr_positions[j] - prev_positions[j];
                seg_len += diff * diff;
            }
            length += std::sqrt(seg_len);
        }
        return length;
    };

    // Compute Cartesian path length
    auto computeCartesianPathLength = [&](const robot_trajectory::RobotTrajectory &traj) -> double
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.getWayPointCount(); ++i)
        {
            const auto &prev = traj.getWayPoint(i - 1);
            const auto &curr = traj.getWayPoint(i);
            Eigen::Isometry3d prev_pose = prev.getGlobalLinkTransform(tcp_frame_);
            Eigen::Isometry3d curr_pose = curr.getGlobalLinkTransform(tcp_frame_);
            double dist = (curr_pose.translation() - prev_pose.translation()).norm();
            length += dist;
        }
        return length;
    };

    double joint_length = computeJointPathLength(trajectory);
    double cart_length = computeCartesianPathLength(trajectory);

    total_length = (2 * cart_length) + (joint_length / 2.0);
    return total_length;
}

// Compute Max Cartesian Speed
double ManyMovePlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
{
    if (trajectory->getWayPointCount() < 2) return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame_);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(tcp_frame_);
        double dist = (curr_pose.translation() - prev_pose.translation()).norm();
        double dt = trajectory->getWayPointDurationFromPrevious(i);
        if (dt > 0.0)
        {
            double speed = dist / dt;
            if (speed > max_speed) max_speed = speed;
        }
    }
    return max_speed;
}

// Apply Time Parameterization
bool ManyMovePlanner::applyTimeParameterization(
    robot_trajectory::RobotTrajectoryPtr &trajectory, const MovementConfig &config)
{
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    const int max_iterations = 5;
    for (int iteration = 0; iteration < max_iterations; iteration++)
    {
        // Reset durations
        for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
            trajectory->setWayPointDurationFromPrevious(i, 0.0);

        bool time_param_success = false;
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
            trajectory_processing::IterativeParabolicTimeParameterization time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            RCLCPP_ERROR(logger_, "Failed to compute time stamps using '%s'", config.smoothing_type.c_str());
            return false;
        }

        double max_cartesian_speed = computeMaxCartesianSpeed(trajectory);
        if (max_cartesian_speed <= config.max_cartesian_speed)
        {
            return true; // Success
        }
        else
        {
            double scale = config.max_cartesian_speed / max_cartesian_speed;
            velocity_scaling_factor *= scale;
            // Adjust acceleration similarly (heuristic):
            acceleration_scaling_factor = (acceleration_scaling_factor * scale + acceleration_scaling_factor) / 2.0;

            if (velocity_scaling_factor < 0.01 || acceleration_scaling_factor < 0.01)
            {
                RCLCPP_ERROR(logger_, "Scaling factors too small to limit Cartesian speed.");
                return false;
            }
        }
    }

    RCLCPP_ERROR(logger_, "Failed to limit Cartesian speed after iterations.");
    return false;
}

// Execute Trajectory
bool ManyMovePlanner::executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory)
{
    bool success = moveit_cpp_ptr_->execute(planning_components_->getPlanningGroupName(), trajectory);
    if (success)
    {
        auto tem = moveit_cpp_ptr_->getTrajectoryExecutionManager();
        bool wait_ok = tem->waitForExecution();
        if (!wait_ok || tem->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        {
            RCLCPP_WARN(logger_, "Execution did not complete successfully.");
            success = false;
        }
    }
    if (!success) RCLCPP_ERROR(logger_, "Trajectory execution failed.");
    return success;
}

// Move to Pose Target
bool ManyMovePlanner::moveToPoseTarget(const geometry_msgs::msg::Pose& target_pose, const MovementConfig &config)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = base_frame_;
    pose_stamped.pose = target_pose;

    int retry = 0;
    bool exec_success = false;

    while (retry < config.max_exec_tries && !exec_success)
    {
        planning_components_->setGoal(pose_stamped, tcp_frame_);

        std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
        int attempts = 0;
        while (attempts < config.plan_number_limit && static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            planning_components_->setStartStateToCurrentState();
            auto plan_solution = planning_components_->plan();
            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double length = computePathLength(*plan_solution.trajectory);
                trajectories.emplace_back(plan_solution, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Pose target planning attempt %d failed", attempts + 1);
            }
            attempts++;
        }

        if (trajectories.empty())
        {
            RCLCPP_ERROR(logger_, "No valid pose target trajectory found.");
            return false;
        }

        // Select the trajectory with the minimum path length
        auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
            [](const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &a,
               const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &b) -> bool {
                return a.second < b.second;
            });

        if (!applyTimeParameterization(shortest->first.trajectory, config))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for Pose Target.");
            return false;
        }

        exec_success = executeTrajectory(shortest->first.trajectory);
        retry++;
    }

    return exec_success;
}

// Move to Joint Target
bool ManyMovePlanner::moveToJointTarget(const std::vector<double>& joint_values, const MovementConfig &config)
{
    int retry = 0;
    bool exec_success = false;
    while (retry < config.max_exec_tries && !exec_success)
    {
        // Corrected constructor: pass the RobotModelConstPtr directly
        moveit::core::RobotState goal_state(moveit_cpp_ptr_->getRobotModel());
        goal_state.setJointGroupPositions(planning_components_->getPlanningGroupName(), joint_values);
        goal_state.update();
        planning_components_->setGoal(goal_state);

        std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
        int attempts = 0;
        while (attempts < config.plan_number_limit && static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            planning_components_->setStartStateToCurrentState();
            auto plan_solution = planning_components_->plan();
            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double length = computePathLength(*plan_solution.trajectory);
                trajectories.emplace_back(plan_solution, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Joint target planning attempt %d failed", attempts + 1);
            }
            attempts++;
        }

        if (trajectories.empty())
        {
            RCLCPP_ERROR(logger_, "No valid joint target trajectory found.");
            return false;
        }

        // Select the trajectory with the minimum path length
        auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
            [](const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &a,
               const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &b) -> bool {
                return a.second < b.second;
            });

        if (!applyTimeParameterization(shortest->first.trajectory, config))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for Joint Target.");
            return false;
        }

        exec_success = executeTrajectory(shortest->first.trajectory);
        retry++;
    }
    return exec_success;
}

// Move to Named Target
bool ManyMovePlanner::moveToNamedTarget(const std::string& target_name, const MovementConfig &config)
{
    int retry = 0;
    bool exec_success = false;
    while (retry < config.max_exec_tries && !exec_success)
    {
        planning_components_->setGoal(target_name);

        std::vector<std::pair<moveit_cpp::PlanningComponent::PlanSolution, double>> trajectories;
        int attempts = 0;
        while (attempts < config.plan_number_limit && static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            planning_components_->setStartStateToCurrentState();
            auto plan_solution = planning_components_->plan();
            if (plan_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                double length = computePathLength(*plan_solution.trajectory);
                trajectories.emplace_back(plan_solution, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Named target planning attempt %d failed", attempts + 1);
            }
            attempts++;
        }

        if (trajectories.empty())
        {
            RCLCPP_ERROR(logger_, "No valid named target trajectory found.");
            return false;
        }

        // Select the trajectory with the minimum path length
        auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
            [](const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &a,
               const std::pair<moveit_cpp::PlanningComponent::PlanSolution, double> &b) -> bool {
                return a.second < b.second;
            });

        if (!applyTimeParameterization(shortest->first.trajectory, config))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for Named Target.");
            return false;
        }

        exec_success = executeTrajectory(shortest->first.trajectory);
        retry++;
    }
    return exec_success;
}

// Move Cartesian Path
bool ManyMovePlanner::moveCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const MovementConfig &config,
    double linear_success_tolerance)
{
    int retry = 0;
    bool exec_success = false;
    while (retry < config.max_exec_tries && !exec_success)
    {
        auto robot_model = moveit_cpp_ptr_->getRobotModel();
        auto jmg = robot_model->getJointModelGroup(planning_components_->getPlanningGroupName());
        const auto link_model = robot_model->getLinkModel(tcp_frame_);

        std::vector<std::pair<robot_trajectory::RobotTrajectoryPtr, double>> trajectories;
        int attempts = 0;

        while (attempts < config.plan_number_limit && static_cast<int>(trajectories.size()) < config.plan_number_target)
        {
            planning_components_->setStartStateToCurrentState();
            auto current_state = moveit_cpp_ptr_->getCurrentState();
            std::vector<moveit::core::RobotStatePtr> trajectory_states;
            EigenSTL::vector_Isometry3d waypoints_eigen;
            for (const auto &pose_msg : waypoints)
            {
                Eigen::Isometry3d pose_eigen;
                tf2::fromMsg(pose_msg, pose_eigen);
                waypoints_eigen.push_back(pose_eigen);
            }

            // Compute Cartesian path
            moveit::core::MaxEEFStep max_step(config.step_size, config.step_size * 2);
            moveit::core::JumpThreshold jump_thresh(config.jump_threshold, config.jump_threshold);

            auto validity_callback = [this](moveit::core::RobotState *state, const moveit::core::JointModelGroup *group,
                                            const double *joint_group_variable_values) -> bool
            {
                state->setJointGroupPositions(group, joint_group_variable_values);
                state->update();
                auto planning_scene = moveit_cpp_ptr_->getPlanningSceneMonitor()->getPlanningScene();
                if (planning_scene)
                {
                    return !planning_scene->isStateColliding(*state, group->getName());
                }
                else
                {
                    RCLCPP_WARN(logger_, "Planning scene is not available for collision checking.");
                    return false;
                }
            };

            double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
                current_state.get(), jmg, trajectory_states, link_model,
                waypoints_eigen, true, max_step, jump_thresh, validity_callback);

            if (fraction >= linear_success_tolerance)
            {
                auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, planning_components_->getPlanningGroupName());
                for (const auto &st : trajectory_states) trajectory->addSuffixWayPoint(*st, 0.0);
                double length = computePathLength(*trajectory);
                trajectories.emplace_back(trajectory, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Cartesian path attempt %d failed (%.2f%% achieved)", attempts + 1, fraction * 100.0);
            }
            attempts++;
        }

        if (trajectories.empty())
        {
            RCLCPP_ERROR(logger_, "No valid Cartesian path found.");
            return false;
        }

        // Select the trajectory with the minimum path length
        auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
            [](const std::pair<robot_trajectory::RobotTrajectoryPtr, double> &a,
               const std::pair<robot_trajectory::RobotTrajectoryPtr, double> &b) -> bool {
                return a.second < b.second;
            });

        if (!applyTimeParameterization(shortest->first, config))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for Cartesian Path.");
            return false;
        }

        exec_success = executeTrajectory(shortest->first);
        retry++;
    }

    return exec_success;
}

// Find Collision Object
bool ManyMovePlanner::findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object)
{
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");

    // Wait until the service is available
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger_, "Service /get_planning_scene not available");
        return false;
    }

    // Create request
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    // components=0 fetches the full scene. If you only need certain parts, set accordingly.
    request->components.components = 0;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Failed to call /get_planning_scene service");
        return false;
    }

    auto response = future.get();
    if (!response) {
        RCLCPP_ERROR(logger_, "Empty response from /get_planning_scene");
        return false;
    }

    // Check objects in the planning scene world
    const auto &objects = response->scene.world.collision_objects;
    for (const auto &obj : objects) {
        if (obj.id.find(partial_id) != std::string::npos) {
            found_object = obj;
            return true;
        }
    }

    return false;
}
