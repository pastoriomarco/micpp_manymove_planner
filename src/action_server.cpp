#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "micpp_manymove_planner/micpp_manymove_planner.hpp"
#include "micpp_manymove_planner/action/move_manipulator.hpp"

class MoveManipulatorActionServer : public rclcpp::Node
{
public:
    using MoveManipulator = micpp_manymove_planner::action::MoveManipulator;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;

    MoveManipulatorActionServer(const std::shared_ptr<ManyMovePlanner>& planner)
    : Node("move_action_server"), planner_(planner)
    {
        action_server_ = rclcpp_action::create_server<MoveManipulator>(
            this,
            "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<MoveManipulator>::SharedPtr action_server_;
    std::shared_ptr<ManyMovePlanner> planner_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulator::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(get_logger(), "Received MoveManipulator goal with movement_type: %s", goal->movement_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        // This runs in a separate thread so that we don't block the executor
        std::thread{std::bind(&MoveManipulatorActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Executing MoveManipulator goal");
        auto result = std::make_shared<MoveManipulator::Result>();

        // Construct MovementConfig from goal
        MovementConfig config;
        config.velocity_scaling_factor = goal_handle->get_goal()->config.velocity_scaling_factor;
        config.acceleration_scaling_factor = goal_handle->get_goal()->config.acceleration_scaling_factor;
        config.step_size = goal_handle->get_goal()->config.step_size;
        config.jump_threshold = goal_handle->get_goal()->config.jump_threshold;
        config.max_cartesian_speed = goal_handle->get_goal()->config.max_cartesian_speed;
        config.max_exec_tries = goal_handle->get_goal()->config.max_exec_tries;
        config.plan_number_target = goal_handle->get_goal()->config.plan_number_target;
        config.plan_number_limit = goal_handle->get_goal()->config.plan_number_limit;
        config.smoothing_type = goal_handle->get_goal()->config.smoothing_type;

        bool success = false;
        auto goal = goal_handle->get_goal();

        if (goal->movement_type == "pose")
        {
            success = planner_->moveToPoseTarget(goal->pose_target, config);
        }
        else if (goal->movement_type == "joint")
        {
            success = planner_->moveToJointTarget(goal->joint_values, config);
        }
        else if (goal->movement_type == "named")
        {
            success = planner_->moveToNamedTarget(goal->named_target, config);
        }
        else if (goal->movement_type == "cartesian")
        {
            success = planner_->moveCartesianPath(goal->cartesian_waypoints, config);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown movement_type: %s", goal->movement_type.c_str());
            result->success = false;
            result->message = "Unknown movement_type";
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(), "Goal canceled");
            result->success = false;
            result->message = "Canceled by client";
            goal_handle->canceled(result);
            return;
        }

        if (success) {
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(get_logger(), "Goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }
};
