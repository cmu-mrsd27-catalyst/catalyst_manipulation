#pragma once

#include <string>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <nlohmann/json.hpp>
#include <catalyst_interfaces/action/execute_task.hpp>

namespace catalyst_bt {

/**
 * BT StatefulActionNode that calls an ExecuteTask action server.
 *
 * Input ports:
 *   - "action_name":  ROS action server name (e.g. "/pick")
 *   - "command":      JSON command string (goal)
 *   - "overrides":    JSON fields to merge into command (optional)
 *   - "timeout_sec":  action timeout in seconds (default: 300)
 *
 * Output ports:
 *   - "result":   full JSON result string
 *   - "success":  "true" or "false"
 *
 * Lifecycle:
 *   onStart()   — sends goal to action server
 *   onRunning() — spins node, checks for result or timeout
 *   onHalted()  — cancels the goal
 */
class TaskActionNode : public BT::StatefulActionNode {
public:
    using ExecuteTask = catalyst_interfaces::action::ExecuteTask;
    using GoalHandle  = rclcpp_action::ClientGoalHandle<ExecuteTask>;

    TaskActionNode(const std::string& name, const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr ros_node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr ros_node_;

    // Action client cache (one per action_name)
    std::unordered_map<std::string,
        rclcpp_action::Client<ExecuteTask>::SharedPtr> clients_;

    rclcpp_action::Client<ExecuteTask>::SharedPtr
    get_client(const std::string& action_name);

    // Goal state (reset each onStart)
    std::string current_action_;
    GoalHandle::SharedPtr goal_handle_;
    std::chrono::steady_clock::time_point deadline_;

    std::mutex result_mutex_;
    bool goal_done_     = false;
    bool goal_accepted_ = false;
    bool goal_rejected_ = false;
    bool goal_success_  = false;
    std::string result_str_;

    // Helper to read the command string (handles BT.CPP blackboard quirk)
    std::string get_command_string();
};

}  // namespace catalyst_bt
