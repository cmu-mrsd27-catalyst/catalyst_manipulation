#include "catalyst_bt/set_bool_service_node.hpp"

namespace catalyst_bt {

SetBoolServiceNode::SetBoolServiceNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr ros_node)
    : BT::SyncActionNode(name, config), ros_node_(ros_node) {}

BT::PortsList SetBoolServiceNode::providedPorts() {
    return {
        BT::InputPort<std::string>("service_name", "ROS service name"),
        BT::InputPort<std::string>("value", "true or false"),
    };
}

BT::NodeStatus SetBoolServiceNode::tick() {
    auto service_name = getInput<std::string>("service_name");
    auto value_str = getInput<std::string>("value");

    if (!service_name || !value_str) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Missing ports for SetBoolServiceNode");
        return BT::NodeStatus::FAILURE;
    }

    // Get or create client
    auto it = clients_.find(service_name.value());
    if (it == clients_.end()) {
        clients_[service_name.value()] =
            ros_node_->create_client<std_srvs::srv::SetBool>(service_name.value());
    }
    auto client = clients_[service_name.value()];

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "Service '%s' not available", service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = (value_str.value() == "true");

    RCLCPP_INFO(ros_node_->get_logger(), "[BT] %s → %s: %s",
                name().c_str(), service_name.value().c_str(),
                value_str.value().c_str());

    auto future = client->async_send_request(request);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node_);
        if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
            break;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            RCLCPP_ERROR(ros_node_->get_logger(), "[BT] %s: timeout", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace catalyst_bt
