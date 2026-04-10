#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <catalyst_interfaces/srv/json_command.hpp>

#include "catalyst_bt/json_service_node.hpp"
#include "catalyst_bt/set_bool_service_node.hpp"
#include "catalyst_bt/empty_service_node.hpp"
#include "catalyst_bt/task_action_node.hpp"
#include "catalyst_bt/system_healthy_node.hpp"

using json = nlohmann::json;
using JsonCommand = catalyst_interfaces::srv::JsonCommand;

static BT::NodeStatus run_tree_to_completion(BT::Tree& tree)
{
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Do not call rclcpp::spin_some(node) here: the node is already on a running
    // MultiThreadedExecutor; nested spin_some throws "already been added to an executor".
    return status;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_executor");

    auto pkg_dir = ament_index_cpp::get_package_share_directory("catalyst_bt");
    const std::string orchestration_file = pkg_dir + "/trees/orchestration.xml";

    RCLCPP_INFO(node->get_logger(), "BT orchestration file: %s", orchestration_file.c_str());

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<catalyst_bt::JsonServiceNode>("JsonService", node);
    factory.registerNodeType<catalyst_bt::SetBoolServiceNode>("SetBoolService", node);
    factory.registerNodeType<catalyst_bt::EmptyServiceNode>("EmptyService", node);
    factory.registerNodeType<catalyst_bt::TaskActionNode>("TaskAction", node);
    factory.registerNodeType<catalyst_bt::SystemHealthyNode>("SystemHealthy", node);

    factory.registerBehaviorTreeFromFile(orchestration_file);

    std::mutex exec_mutex;

    // Reentrant group: /bt_execute blocks in run_tree_to_completion; without this, the node's
    // default MutuallyExclusive group would prevent other threads from handling subscriptions
    // and async client/action responses while the service callback runs.
    auto bt_execute_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto srv = node->create_service<JsonCommand>(
        "/bt_execute",
        [&factory, node, &exec_mutex](
            const std::shared_ptr<JsonCommand::Request> req,
            std::shared_ptr<JsonCommand::Response> resp)
        {
            std::lock_guard<std::mutex> lock(exec_mutex);

            std::string task;
            try {
                auto j = json::parse(req->command);
                if (j.contains("task") && j["task"].is_string()) {
                    task = j["task"].get<std::string>();
                }
            } catch (const json::exception& e) {
                resp->response = json{
                    {"success", false},
                    {"message", std::string("Invalid JSON: ") + e.what()},
                }.dump();
                return;
            }

            static const std::unordered_map<std::string, std::string> k_task_to_tree{
                {"pick_only", "MainPickOnly"},
                {"place_only", "MainPlaceOnly"},
                {"pick_place", "MainPickPlace"},
                {"place_pick", "MainPlacePick"},
                {"pick_then_place_on_robot", "MainPickThenPlaceOnRobot"},
                {"pick_from_robot_container", "MainPickFromRobotContainer"},
                {"pick_only_with_detect", "MainPickOnlyWithDetect"},
                {"place_only_with_detect", "MainPlaceOnlyWithDetect"},
                {"pick_place_with_detect", "MainPickPlaceWithDetect"},
                {"place_pick_with_detect", "MainPlacePickWithDetect"},
            };

            auto it = k_task_to_tree.find(task);
            if (it == k_task_to_tree.end()) {
                resp->response = json{
                    {"success", false},
                    {"message", "Unknown task. Use: pick_only | place_only | pick_place | place_pick | "
                                "pick_then_place_on_robot | pick_from_robot_container | "
                                "pick_only_with_detect | place_only_with_detect | pick_place_with_detect | "
                                "place_pick_with_detect"},
                    {"task", task},
                }.dump();
                return;
            }

            const std::string& tree_id = it->second;
            RCLCPP_INFO(node->get_logger(), "Running BT tree: %s (task=%s)",
                        tree_id.c_str(), task.c_str());

            BT::Tree tree;
            try {
                tree = factory.createTree(tree_id);
            } catch (const std::exception& e) {
                resp->response = json{
                    {"success", false},
                    {"message", std::string("createTree failed: ") + e.what()},
                    {"task", task},
                }.dump();
                return;
            }

            const BT::NodeStatus status = run_tree_to_completion(tree);
            const bool ok = (status == BT::NodeStatus::SUCCESS);

            resp->response = json{
                {"success", ok},
                {"task", task},
                {"tree_id", tree_id},
                {"bt_status", ok ? "SUCCESS" : "FAILURE"},
            }.dump();

            if (ok) {
                RCLCPP_INFO(node->get_logger(), "BT finished: SUCCESS (%s)", task.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "BT finished: FAILURE (%s)", task.c_str());
            }
        },
        rmw_qos_profile_services_default,
        bt_execute_cb_group);

    (void)srv;

    RCLCPP_INFO(node->get_logger(),
                "bt_executor ready — call: ros2 service call /bt_execute "
                "catalyst_interfaces/srv/JsonCommand \"{command: '{\\\"task\\\": \\\"pick_only\\\"}'}\" "
                "(also: pick_then_place_on_robot | pick_from_robot_container)");

    // Multi-threaded executor: /bt_execute handler blocks in run_tree_to_completion while
    // ticking the BT; other threads must process subscriptions, service clients, and action
    // clients. rclcpp::spin_some(shared_node) is invalid here (node already on this executor).
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4u);
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
