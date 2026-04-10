#include "catalyst_bt/system_healthy_node.hpp"

namespace catalyst_bt {

SystemHealthyNode::SystemHealthyNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr ros_node)
    : BT::ConditionNode(name, config),
      ros_node_(ros_node),
      last_msg_time_(std::chrono::steady_clock::now())
{
    auto topic = getInput<std::string>("topic").value_or("/world_model");

    sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        topic, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            _world_model_cb(msg);
        });
}

BT::PortsList SystemHealthyNode::providedPorts() {
    return {
        BT::InputPort<std::string>("topic", "/world_model", "World model topic"),
        BT::InputPort<double>("timeout_sec", 2.0, "Max age of world model msg"),
    };
}

void SystemHealthyNode::_world_model_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_msg_time_ = std::chrono::steady_clock::now();

    try {
        auto j = nlohmann::json::parse(msg->data);
        auto health = j.value("system_health", nlohmann::json::object());
        healthy_ = health.value("healthy", true);

        if (!healthy_) {
            // Find the first failing check for logging
            auto checks = health.value("checks", nlohmann::json::object());
            last_unhealthy_reason_.clear();
            for (auto& [name, check] : checks.items()) {
                bool ok = check.value("ok", true);
                bool suppressed = check.value("suppressed", false);
                bool disabled = check.value("disabled", false);
                if (!ok && !suppressed && !disabled) {
                    last_unhealthy_reason_ = name + ": " +
                        check.value("msg", "unknown");
                    break;
                }
            }
        }
    } catch (const nlohmann::json::exception& e) {
        // Parse failure — don't mark unhealthy for transient JSON issues
        RCLCPP_WARN_THROTTLE(ros_node_->get_logger(),
                             *ros_node_->get_clock(), 5000,
                             "[BT] SystemHealthy: JSON parse error: %s",
                             e.what());
    }
}

BT::NodeStatus SystemHealthyNode::tick() {
    // Do not call rclcpp::spin_some(ros_node_) — node is on MultiThreadedExecutor; other
    // threads process /world_model subscription while /bt_execute ticks the tree.

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Check if world model is stale
    double timeout = getInput<double>("timeout_sec").value_or(2.0);
    auto age = std::chrono::steady_clock::now() - last_msg_time_;
    double age_sec = std::chrono::duration<double>(age).count();

    if (age_sec > timeout) {
        RCLCPP_WARN_THROTTLE(ros_node_->get_logger(),
                             *ros_node_->get_clock(), 5000,
                             "[BT] SystemHealthy: world model stale (%.1fs)",
                             age_sec);
        return BT::NodeStatus::FAILURE;
    }

    if (!healthy_) {
        RCLCPP_WARN_THROTTLE(ros_node_->get_logger(),
                             *ros_node_->get_clock(), 2000,
                             "[BT] SystemHealthy: UNHEALTHY — %s",
                             last_unhealthy_reason_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace catalyst_bt
