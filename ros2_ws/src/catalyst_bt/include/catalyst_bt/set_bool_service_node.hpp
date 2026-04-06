#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_srvs/srv/set_bool.hpp>

namespace catalyst_bt {

/**
 * BT action node that calls a SetBool service.
 *
 * Input ports:
 *   - "service_name": ROS service to call
 *   - "value": "true" or "false"
 */
class SetBoolServiceNode : public BT::SyncActionNode {
public:
    SetBoolServiceNode(const std::string& name, const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr ros_node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr ros_node_;
    std::unordered_map<std::string,
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> clients_;
};

}  // namespace catalyst_bt
