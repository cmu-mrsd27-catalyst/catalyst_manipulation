#pragma once

#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

namespace catalyst_bt {

/**
 * BT Condition node that checks system health from /world_model.
 *
 * Subscribes to /world_model (JSON), reads system_health.healthy.
 * Returns SUCCESS if healthy, FAILURE otherwise.
 *
 * Input ports:
 *   - "topic": topic name (default: "/world_model")
 *   - "timeout_sec": max age of last message before unhealthy (default: 2.0)
 */
class SystemHealthyNode : public BT::ConditionNode {
public:
    SystemHealthyNode(const std::string& name, const BT::NodeConfiguration& config,
                      rclcpp::Node::SharedPtr ros_node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    std::mutex data_mutex_;
    bool healthy_ = true;
    std::chrono::steady_clock::time_point last_msg_time_;
    std::string last_unhealthy_reason_;

    void _world_model_cb(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace catalyst_bt
