#pragma once

#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <nlohmann/json.hpp>
#include <catalyst_interfaces/srv/json_command.hpp>

namespace catalyst_bt {

/**
 * Generic BT action node that calls a JsonCommand service.
 *
 * Input ports:
 *   - "service_name": ROS service to call (e.g. "/joint_command")
 *   - "command": JSON string to send as the command
 *   - "timeout_sec": service call timeout in seconds (default: 60)
 *
 * Output ports:
 *   - "response": full JSON response string
 *   - "success": "true" or "false"
 *
 * The node caches service clients per service_name for reuse.
 */
class JsonServiceNode : public BT::SyncActionNode {
public:
    JsonServiceNode(const std::string& name, const BT::NodeConfiguration& config,
                    rclcpp::Node::SharedPtr ros_node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr ros_node_;
    std::unordered_map<std::string,
        rclcpp::Client<catalyst_interfaces::srv::JsonCommand>::SharedPtr> clients_;

    rclcpp::Client<catalyst_interfaces::srv::JsonCommand>::SharedPtr
    get_client(const std::string& service_name);
};

}  // namespace catalyst_bt
