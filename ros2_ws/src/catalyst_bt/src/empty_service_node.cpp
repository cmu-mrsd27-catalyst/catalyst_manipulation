#include "catalyst_bt/empty_service_node.hpp"

namespace catalyst_bt {

EmptyServiceNode::EmptyServiceNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr ros_node)
    : BT::SyncActionNode(name, config), ros_node_(ros_node) {}

BT::PortsList EmptyServiceNode::providedPorts() {
    return {
        BT::InputPort<std::string>("service_name", "ROS service name"),
    };
}

BT::NodeStatus EmptyServiceNode::tick() {
    auto service_name = getInput<std::string>("service_name");

    if (!service_name) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Missing 'service_name' port");
        return BT::NodeStatus::FAILURE;
    }

    auto it = clients_.find(service_name.value());
    if (it == clients_.end()) {
        clients_[service_name.value()] =
            ros_node_->create_client<std_srvs::srv::Empty>(service_name.value());
    }
    auto client = clients_[service_name.value()];

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "Service '%s' not available", service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "[BT] %s → %s",
                name().c_str(), service_name.value().c_str());

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
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
