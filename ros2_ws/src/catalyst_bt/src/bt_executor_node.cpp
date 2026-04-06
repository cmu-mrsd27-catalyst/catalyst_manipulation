#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "catalyst_bt/json_service_node.hpp"
#include "catalyst_bt/set_bool_service_node.hpp"
#include "catalyst_bt/empty_service_node.hpp"
#include "catalyst_bt/task_action_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_executor");

    // Declare parameters
    node->declare_parameter<std::string>("tree_file", "");

    std::string tree_file;
    node->get_parameter("tree_file", tree_file);

    // If no absolute path given, look in package share
    if (tree_file.empty() || tree_file[0] != '/') {
        auto pkg_dir = ament_index_cpp::get_package_share_directory("catalyst_bt");
        if (tree_file.empty()) {
            tree_file = pkg_dir + "/trees/pick_and_place.xml";
        } else {
            tree_file = pkg_dir + "/trees/" + tree_file;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", tree_file.c_str());

    // Create BT factory and register node types
    BT::BehaviorTreeFactory factory;

    // JsonServiceNode — generic JsonCommand service caller
    // Used for: /joint_command, /cartesian_command, /gripper_command,
    //           /guide_mode, /sdk_control, /compute_poses, /explore_tag
    factory.registerNodeType<catalyst_bt::JsonServiceNode>(
        "JsonService",
        node  // pass ROS node to constructor
    );

    // SetBoolServiceNode — for /set_octomap_enabled
    factory.registerNodeType<catalyst_bt::SetBoolServiceNode>(
        "SetBoolService",
        node
    );

    // EmptyServiceNode — for /clear_octomap
    factory.registerNodeType<catalyst_bt::EmptyServiceNode>(
        "EmptyService",
        node
    );

    // TaskActionNode — for ExecuteTask action servers (/pick, /place, /explore)
    factory.registerNodeType<catalyst_bt::TaskActionNode>(
        "TaskAction",
        node
    );

    // Load and create tree
    factory.registerBehaviorTreeFromFile(tree_file);
    auto tree = factory.createTree("MainTree");

    RCLCPP_INFO(node->get_logger(), "BT loaded. Ticking...");

    // Tick the tree to completion
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "BT completed: SUCCESS");
    } else {
        RCLCPP_ERROR(node->get_logger(), "BT completed: FAILURE");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
