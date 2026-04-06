#include "catalyst_bt/task_action_node.hpp"

namespace catalyst_bt {

TaskActionNode::TaskActionNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr ros_node)
    : BT::StatefulActionNode(name, config), ros_node_(ros_node) {}

BT::PortsList TaskActionNode::providedPorts() {
    return {
        BT::InputPort<std::string>("action_name", "ROS action server name"),
        BT::InputPort<std::string>("command", "", "JSON command string"),
        BT::InputPort<std::string>("overrides", "", "JSON fields to merge"),
        BT::InputPort<double>("timeout_sec", 300.0, "Action timeout"),
        BT::OutputPort<std::string>("result", "JSON result string"),
        BT::OutputPort<std::string>("success", "true/false"),
    };
}

rclcpp_action::Client<TaskActionNode::ExecuteTask>::SharedPtr
TaskActionNode::get_client(const std::string& action_name) {
    auto it = clients_.find(action_name);
    if (it != clients_.end()) {
        return it->second;
    }
    auto client = rclcpp_action::create_client<ExecuteTask>(ros_node_, action_name);
    clients_[action_name] = client;
    return client;
}

std::string TaskActionNode::get_command_string() {
    // BT.CPP interprets strings starting with '{' as blackboard references.
    // Try getInput first (handles blackboard vars), fall back to raw XML attr.
    std::string cmd_str;
    auto result = getInput<std::string>("command");
    if (result && !result.value().empty()) {
        cmd_str = result.value();
    } else {
        auto raw = config().input_ports.find("command");
        if (raw != config().input_ports.end()) {
            cmd_str = raw->second;
        }
    }

    // Merge overrides if provided
    std::string overrides_str;
    auto ov_result = getInput<std::string>("overrides");
    if (ov_result && !ov_result.value().empty()) {
        overrides_str = ov_result.value();
    } else {
        auto raw_ov = config().input_ports.find("overrides");
        if (raw_ov != config().input_ports.end()) {
            overrides_str = raw_ov->second;
        }
    }
    if (!overrides_str.empty()) {
        try {
            auto base = nlohmann::json::parse(cmd_str.empty() ? "{}" : cmd_str);
            auto ovr = nlohmann::json::parse(overrides_str);
            base.merge_patch(ovr);
            cmd_str = base.dump();
        } catch (const nlohmann::json::exception& e) {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "[BT] %s: failed to merge overrides: %s",
                        name().c_str(), e.what());
        }
    }

    return cmd_str.empty() ? "{}" : cmd_str;
}

// ── Lifecycle ──

BT::NodeStatus TaskActionNode::onStart() {
    auto action_name = getInput<std::string>("action_name");
    if (!action_name || action_name.value().empty()) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "[BT] %s: missing 'action_name' port", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    current_action_ = action_name.value();

    auto client = get_client(current_action_);

    if (!client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "[BT] %s: action server '%s' not available",
                     name().c_str(), current_action_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Reset state
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        goal_done_ = false;
        goal_accepted_ = false;
        goal_rejected_ = false;
        goal_success_ = false;
        result_str_.clear();
        goal_handle_.reset();
    }

    // Build goal
    auto goal = ExecuteTask::Goal();
    goal.command = get_command_string();

    RCLCPP_INFO(ros_node_->get_logger(), "[BT] %s -> %s: %s",
                name().c_str(), current_action_.c_str(),
                goal.command.c_str());

    // Set up callbacks
    auto send_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();

    send_options.goal_response_callback =
        [this](const GoalHandle::SharedPtr& gh) {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (!gh) {
                RCLCPP_ERROR(ros_node_->get_logger(),
                             "[BT] %s: goal rejected by '%s'",
                             name().c_str(), current_action_.c_str());
                goal_rejected_ = true;
                goal_done_ = true;
            } else {
                goal_accepted_ = true;
                goal_handle_ = gh;
            }
        };

    send_options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
            try {
                auto fb = nlohmann::json::parse(feedback->feedback);
                RCLCPP_INFO(ros_node_->get_logger(),
                            "[BT] %s feedback: %s — %s",
                            name().c_str(),
                            fb.value("phase", "").c_str(),
                            fb.value("message", "").c_str());
            } catch (...) {
                RCLCPP_INFO(ros_node_->get_logger(),
                            "[BT] %s feedback: %s",
                            name().c_str(), feedback->feedback.c_str());
            }
        };

    send_options.result_callback =
        [this](const GoalHandle::WrappedResult& wrapped) {
            std::lock_guard<std::mutex> lock(result_mutex_);
            goal_done_ = true;
            result_str_ = wrapped.result->response;

            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
                try {
                    auto resp = nlohmann::json::parse(result_str_);
                    goal_success_ = resp.value("success", false);
                } catch (...) {
                    goal_success_ = false;
                }
            } else {
                goal_success_ = false;
            }

            RCLCPP_INFO(ros_node_->get_logger(),
                        "[BT] %s result: %s (code=%d)",
                        name().c_str(), result_str_.c_str(),
                        static_cast<int>(wrapped.code));
        };

    // Send goal (non-blocking)
    client->async_send_goal(goal, send_options);

    // Set deadline
    double timeout = getInput<double>("timeout_sec").value_or(300.0);
    deadline_ = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000));

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TaskActionNode::onRunning() {
    // Process ROS callbacks so goal_response / feedback / result arrive
    rclcpp::spin_some(ros_node_);

    std::lock_guard<std::mutex> lock(result_mutex_);

    if (goal_done_) {
        setOutput("result", result_str_);
        setOutput("success", goal_success_ ? "true" : "false");
        return goal_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    if (std::chrono::steady_clock::now() > deadline_) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "[BT] %s: timeout waiting for '%s'",
                     name().c_str(), current_action_.c_str());
        // Best-effort cancel
        if (goal_handle_) {
            auto client = get_client(current_action_);
            client->async_cancel_goal(goal_handle_);
        }
        setOutput("success", "false");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void TaskActionNode::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(),
                "[BT] %s: halted, canceling goal on '%s'",
                name().c_str(), current_action_.c_str());

    std::lock_guard<std::mutex> lock(result_mutex_);
    if (goal_handle_) {
        auto client = get_client(current_action_);
        client->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    goal_done_ = false;
}

}  // namespace catalyst_bt
