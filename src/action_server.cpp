// src/micpp_manymove_planner/src/action_server.cpp

#include "micpp_manymove_planner/micpp_manymove_planner.hpp"
#include "micpp_manymove_planner/action/move_manipulator.hpp"
#include "micpp_manymove_planner/action/move_manipulator_sequence.hpp"
#include "micpp_manymove_planner/msg/movement_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

// Using declarations outside the class for proper namespace access
using micpp_manymove_planner::msg::MovementConfig;
using micpp_manymove_planner::action::MoveManipulator;
using micpp_manymove_planner::action::MoveManipulatorSequence;
using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

class MoveManipulatorActionServer
{
public:
    MoveManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<ManyMovePlanner> &planner)
        : node_(node), planner_(planner)
    {
        single_action_server_ = rclcpp_action::create_server<MoveManipulator>(
            node_,
            "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_single_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_single_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_single_accepted, this, std::placeholders::_1));

        sequence_action_server_ = rclcpp_action::create_server<MoveManipulatorSequence>(
            node_,
            "move_manipulator_sequence",
            std::bind(&MoveManipulatorActionServer::handle_sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_sequence_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_sequence_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ManyMovePlanner> planner_;

    rclcpp_action::Server<MoveManipulator>::SharedPtr single_action_server_;
    rclcpp_action::Server<MoveManipulatorSequence>::SharedPtr sequence_action_server_;

    // Single Move Manipulator Goal Handling
    rclcpp_action::GoalResponse handle_single_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulator::Goal> goal)
    {
        (void)uuid;

        if (goal->goals.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Received single MoveManipulator goal with empty goals array.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(node_->get_logger(), "Received single MoveManipulator goal with movement_type: %s", goal->goals[0].movement_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_single_cancel(
        const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel single move goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_single_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        // Execute in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_single, this, goal_handle)}.detach();
    }

    void execute_single(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing single MoveManipulator goal");
        auto result = std::make_shared<MoveManipulator::Result>();

        const auto &goal_msg = goal_handle->get_goal();

        if (goal_msg->goals.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Single MoveManipulator goal has no goals.");
            result->success = false;
            result->message = "No goals provided.";
            goal_handle->abort(result);
            return;
        }

        const auto &single_goal = goal_msg->goals[0];
        const MovementConfig &config = single_goal.config;

        bool success = false;

        if (single_goal.movement_type == "pose")
        {
            success = planner_->moveToPoseTarget(single_goal.pose_target, config);
        }
        else if (single_goal.movement_type == "joint")
        {
            success = planner_->moveToJointTarget(single_goal.joint_values, config);
        }
        else if (single_goal.movement_type == "named")
        {
            success = planner_->moveToNamedTarget(single_goal.named_target, config);
        }
        else if (single_goal.movement_type == "cartesian")
        {
            // Assuming cartesian_waypoints is a std::vector<geometry_msgs::msg::Pose>
            success = planner_->moveCartesianPath(single_goal.cartesian_waypoints, config, 0.99);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown movement_type: %s", single_goal.movement_type.c_str());
            result->success = false;
            result->message = "Unknown movement_type";
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(node_->get_logger(), "Goal canceled");
            result->success = false;
            result->message = "Canceled by client";
            goal_handle->canceled(result);
            return;
        }

        if (success)
        {
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }

    // Move Manipulator Sequence Goal Handling
    rclcpp_action::GoalResponse handle_sequence_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulatorSequence::Goal> goal)
    {
        (void)uuid;
        if (goal->goals.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Received MoveManipulatorSequence goal with empty goals array.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(node_->get_logger(), "Received MoveManipulatorSequence goal with %zu moves", goal->goals.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_sequence_cancel(
        const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel sequence goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_sequence_accepted(const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        // Execute in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_sequence, this, goal_handle)}.detach();
    }

    void execute_sequence(const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing MoveManipulatorSequence goal");
        auto result = std::make_shared<MoveManipulatorSequence::Result>();

        const auto &sequence_goal = goal_handle->get_goal();

        if (sequence_goal->goals.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Sequence MoveManipulator goal has no goals.");
            result->success = false;
            result->message = "No goals provided in sequence.";
            goal_handle->abort(result);
            return;
        }

        // Plan the sequence
        std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
        std::vector<MovementConfig> configs;

        for (const auto &goal : sequence_goal->goals)
        {
            // Plan each move
            auto [success, trajectory] = planner_->plan(goal);
            if (!success)
            {
                RCLCPP_ERROR(node_->get_logger(), "Planning failed for a move in sequence.");
                result->success = false;
                result->message = "Planning failed for a move in sequence.";
                goal_handle->abort(result);
                return;
            }
            trajectories.push_back(trajectory);
            configs.push_back(goal.config);
        }

        // Apply time parametrization
        std::vector<size_t> sizes;
        auto [param_success, final_trajectory] = planner_->applyTimeParametrizationSequence(trajectories, configs, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time parametrization sequence failed.");
            result->success = false;
            result->message = "Time parametrization sequence failed.";
            goal_handle->abort(result);
            return;
        }

        // Execute WITH feedback for the sequence
        bool exec_success = planner_->executeTrajectoryWithFeedback(final_trajectory, sizes, goal_handle);
        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Sequence goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Sequence goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }
};
