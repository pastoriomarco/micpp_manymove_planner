// src/micpp_manymove_planner/src/action_server_node.cpp

#include <rclcpp/rclcpp.hpp>
#include "micpp_manymove_planner/micpp_manymove_planner.hpp"
#include "action_server.cpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("action_server_node", "", node_options);

    // Get parameters or set defaults
    std::string planning_group;
    node->get_parameter_or<std::string>("planning_group", planning_group, "lite6");
    std::string base_frame;
    node->get_parameter_or<std::string>("base_frame", base_frame, "link_base");
    std::string tcp_frame;
    node->get_parameter_or<std::string>("tcp_frame", tcp_frame, "link_tcp");

    // Initialize the planner
    auto planner = std::make_shared<ManyMovePlanner>(node, planning_group, base_frame, tcp_frame);

    // Initialize the action server
    auto action_server = std::make_shared<MoveManipulatorActionServer>(planner);

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
