#include "catalyst_bt/json_service_node.hpp"

#include <chrono>
#include <thread>

namespace catalyst_bt {

JsonServiceNode::JsonServiceNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr ros_node)
    : BT::SyncActionNode(name, config), ros_node_(ros_node) {}

BT::PortsList JsonServiceNode::providedPorts() {
    return {
        BT::InputPort<std::string>("service_name", "ROS service name"),
        BT::InputPort<std::string>("command", "", "JSON command string"),
        BT::InputPort<std::string>("overrides", "", "JSON fields to merge into command"),
        BT::InputPort<double>("timeout_sec", 60.0, "Service call timeout"),
        BT::OutputPort<std::string>("response", "JSON response string"),
        BT::OutputPort<std::string>("success", "true/false"),
    };
}

rclcpp::Client<catalyst_interfaces::srv::JsonCommand>::SharedPtr
JsonServiceNode::get_client(const std::string& service_name) {
    auto it = clients_.find(service_name);
    if (it != clients_.end()) {
        return it->second;
    }
    auto client = ros_node_->create_client<catalyst_interfaces::srv::JsonCommand>(service_name);
    clients_[service_name] = client;
    return client;
}

BT::NodeStatus JsonServiceNode::tick() {
    auto service_name = getInput<std::string>("service_name");
    auto timeout_sec = getInput<double>("timeout_sec");

    if (!service_name) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Missing 'service_name' port");
        return BT::NodeStatus::FAILURE;
    }

    // Get the command string. getInput may fail for literal JSON because
    // BT.CPP interprets strings starting with '{' as blackboard references.
    // Fall back to the raw XML attribute value in that case.
    std::string cmd_str;
    auto command_result = getInput<std::string>("command");
    if (command_result && !command_result.value().empty()) {
        cmd_str = command_result.value();
    } else {
        // Read raw attribute from the XML
        auto raw = config().input_ports.find("command");
        if (raw != config().input_ports.end()) {
            cmd_str = raw->second;
        }
        if (cmd_str.empty()) {
            cmd_str = "{}";
        }
    }

    // Merge overrides into the command JSON if provided
    std::string overrides_str;
    auto overrides_result = getInput<std::string>("overrides");
    if (overrides_result && !overrides_result.value().empty()) {
        overrides_str = overrides_result.value();
    } else {
        auto raw_ov = config().input_ports.find("overrides");
        if (raw_ov != config().input_ports.end()) {
            overrides_str = raw_ov->second;
        }
    }
    if (!overrides_str.empty()) {
        try {
            auto base = nlohmann::json::parse(cmd_str);
            auto ovr = nlohmann::json::parse(overrides_str);
            base.merge_patch(ovr);
            cmd_str = base.dump();
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "[BT] %s: failed to merge overrides: %s",
                        name().c_str(), e.what());
        }
    }

    auto client = get_client(service_name.value());

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "Service '%s' not available", service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<catalyst_interfaces::srv::JsonCommand::Request>();
    request->command = cmd_str;

    RCLCPP_INFO(ros_node_->get_logger(), "[BT] %s → %s: %s",
                name().c_str(), service_name.value().c_str(),
                request->command.c_str());

    auto future = client->async_send_request(request);

    double timeout = timeout_sec.value_or(60.0);
    auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000));

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
            break;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            RCLCPP_ERROR(ros_node_->get_logger(),
                         "[BT] %s: timeout calling '%s'",
                         name().c_str(), service_name.value().c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    auto response = future.get();
    std::string resp_str = response->response;

    RCLCPP_INFO(ros_node_->get_logger(), "[BT] %s ← %s",
                name().c_str(), resp_str.c_str());

    setOutput("response", resp_str);

    // Parse success field from JSON response
    try {
        auto resp_json = nlohmann::json::parse(resp_str);
        bool ok = resp_json.value("success", false);
        setOutput("success", ok ? "true" : "false");
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN(ros_node_->get_logger(),
                    "[BT] %s: failed to parse response JSON: %s",
                    name().c_str(), e.what());
        setOutput("success", "false");
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace catalyst_bt
