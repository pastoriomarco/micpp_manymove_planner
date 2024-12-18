#ifndef MICPP_MANYMOVE_PLANNER_HPP
#define MICPP_MANYMOVE_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "micpp_manymove_planner/action/move_manipulator.hpp"
#include "micpp_manymove_planner/action/move_manipulator_sequence.hpp"
#include "micpp_manymove_planner/msg/movement_config.hpp"
#include "micpp_manymove_planner/msg/move_manipulator_goal.hpp"

class ManyMovePlanner
{
public:
    ManyMovePlanner(
        const rclcpp::Node::SharedPtr& node,
        const std::string& planning_group,
        const std::string& base_frame,
        const std::string& tcp_frame);

    bool applyTimeParameterization(
        robot_trajectory::RobotTrajectoryPtr &trajectory,
        const micpp_manymove_planner::msg::MovementConfig &config);

    bool executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory);

    bool moveToPoseTarget(const geometry_msgs::msg::Pose& target_pose, const micpp_manymove_planner::msg::MovementConfig &config);
    bool moveToJointTarget(const std::vector<double>& joint_values, const micpp_manymove_planner::msg::MovementConfig &config);
    bool moveToNamedTarget(const std::string& target_name, const micpp_manymove_planner::msg::MovementConfig &config);
    bool moveCartesianPath(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const micpp_manymove_planner::msg::MovementConfig &config,
        double linear_success_tolerance = 0.99);

    bool findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object);

private:

    double computePathLength(const robot_trajectory::RobotTrajectory &trajectory) const;
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string tcp_frame_;

    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
};

#endif // MICPP_MANYMOVE_PLANNER_HPP
