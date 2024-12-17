#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>

struct MovementConfig
{
    double velocity_scaling_factor = 0.5;
    double acceleration_scaling_factor = 0.5;
    double step_size = 0.01;
    double jump_threshold = 0.0;
    double max_cartesian_speed = 0.5;
    int max_exec_tries = 5;
    int plan_number_target = 12;
    int plan_number_limit = 32;
    std::string smoothing_type = "iterative_parabolic";
};

class ManyMovePlanner
{
public:
    ManyMovePlanner(
        const rclcpp::Node::SharedPtr& node,
        const std::string& planning_group,
        const std::string& base_frame,
        const std::string& tcp_frame);

    bool moveToPoseTarget(const geometry_msgs::msg::Pose& target_pose, const MovementConfig& config);
    bool moveToJointTarget(const std::vector<double>& joint_values, const MovementConfig& config);
    bool moveToNamedTarget(const std::string& target_name, const MovementConfig& config);
    bool moveCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                           const MovementConfig& config,
                           double linear_success_tolerance = 0.99);

    bool findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object);

private:
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const MovementConfig &config);
    bool executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory);
    double computePathLength(const robot_trajectory::RobotTrajectory &trajectory) const;
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string tcp_frame_;
    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    moveit_cpp::PlanningComponentPtr planning_components_;
};
