#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <catalyst_interfaces/srv/gripper_command.hpp>
#include <nlohmann/json.hpp>

#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <atomic>
#include <limits>
#include <cstdlib>

using json = nlohmann::json;
using GripperCommand = catalyst_interfaces::srv::GripperCommand;

const std::string PLANNING_GROUP_ARM = "xarm6";
const std::string PLANNING_GROUP_GRIPPER = "bio_gripper";
const std::string BASE_FRAME = "link_base";
const std::string EE_LINK = "link_tcp";
const std::vector<std::string> JOINT_NAMES = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // 1. CREATE TWO SEPARATE NODES
    // service_node handles your custom network calls.
    // moveit_node is dedicated purely to the MoveGroupInterface to prevent executor clashes.
    auto service_node = rclcpp::Node::make_shared("motion_planner", node_options);
    auto moveit_node = rclcpp::Node::make_shared("motion_planner_moveit", node_options);
    auto logger = service_node->get_logger();

    // 2. SETUP THE EXECUTOR WITH BOTH NODES
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(service_node);
    executor.add_node(moveit_node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    bool enable_gripper = service_node->get_parameter_or("enable_gripper_service", rclcpp::Parameter("enable_gripper_service", true)).as_bool();

    // 3. INITIALIZE MOVEIT
    RCLCPP_INFO(logger, "Initializing MoveGroupInterface (this may take a moment to sync clocks)...");
    auto arm = moveit::planning_interface::MoveGroupInterface(moveit_node, PLANNING_GROUP_ARM);

    arm.setPoseReferenceFrame(BASE_FRAME);
    arm.setEndEffectorLink(EE_LINK);
    // arm.setPlanningTime(30.0);
    // arm.setNumPlanningAttempts(10);

    RCLCPP_INFO(logger, "MoveGroupInterface ready for '%s'", PLANNING_GROUP_ARM.c_str());

    // ===================== Joint Service =====================
    auto joint_service = service_node->create_service<GripperCommand>(
        "/joint_command",
        [&arm, &logger](
            const GripperCommand::Request::SharedPtr request,
            GripperCommand::Response::SharedPtr response)
        {
            try {
                auto cmd = json::parse(request->command);

                std::vector<double> joints_rad(6);

                if (cmd.contains("pose")) {
                    // Named pose
                    std::string pose_name = cmd["pose"].get<std::string>();
                    if (!arm.setNamedTarget(pose_name)) {
                        response->response = json({{"success", false},
                            {"message", "Unknown pose: " + pose_name}}).dump();
                        return;
                    }
                    RCLCPP_INFO(logger, "Joint goal: named pose '%s'", pose_name.c_str());
                } else if (cmd.contains("joints")) {
                    // Joint angles in degrees
                    auto joints_deg = cmd["joints"].get<std::vector<double>>();
                    if (joints_deg.size() != 6) {
                        response->response = json({{"success", false},
                            {"message", "Expected 6 joint values, got " + std::to_string(joints_deg.size())}}).dump();
                        return;
                    }
                    for (size_t i = 0; i < 6; i++) {
                        joints_rad[i] = joints_deg[i] * M_PI / 180.0;
                    }
                    arm.setJointValueTarget(joints_rad);
                    RCLCPP_INFO(logger, "Joint goal: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                                joints_deg[0], joints_deg[1], joints_deg[2],
                                joints_deg[3], joints_deg[4], joints_deg[5]);
                } else {
                    response->response = json({{"success", false},
                        {"message", "Provide 'joints' (6 angles in degrees) or 'pose' (name)"}}).dump();
                    return;
                }

                // Speed scaling
                double speed = cmd.value("speed", 1.0);
                speed = std::clamp(speed, 0.01, 1.0);
                arm.setMaxVelocityScalingFactor(speed);
                arm.setMaxAccelerationScalingFactor(speed);

                // Plan and execute
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) {
                    arm.execute(plan);
                    response->response = json({{"success", true},
                        {"message", "Joint motion succeeded"}}).dump();
                } else {
                    response->response = json({{"success", false},
                        {"message", "Planning failed"}}).dump();
                }

            } catch (const std::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    // ===================== Cartesian Service =====================
    auto cartesian_service = service_node->create_service<GripperCommand>(
        "/cartesian_command",
        [&arm, &logger](
            const GripperCommand::Request::SharedPtr request,
            GripperCommand::Response::SharedPtr response)
        {
            try {
                auto cmd = json::parse(request->command);

                geometry_msgs::msg::Pose target;
                target.position.x = cmd.at("x").get<double>();
                target.position.y = cmd.at("y").get<double>();
                target.position.z = cmd.at("z").get<double>();
                target.orientation.x = cmd.at("qx").get<double>();
                target.orientation.y = cmd.at("qy").get<double>();
                target.orientation.z = cmd.at("qz").get<double>();
                target.orientation.w = cmd.at("qw").get<double>();

                double speed = cmd.value("speed", 1.0);
                speed = std::clamp(speed, 0.01, 1.0);
                arm.setMaxVelocityScalingFactor(speed);
                arm.setMaxAccelerationScalingFactor(speed);

                bool keep_orientation = cmd.value("keep_orientation", false);
                bool straight_line = cmd.value("straight_line", false);

                RCLCPP_INFO(logger, "Cartesian goal: pos=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f) straight=%d orient=%d",
                            target.position.x, target.position.y, target.position.z,
                            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w,
                            straight_line, keep_orientation);

                // Straight-line uses computeCartesianPath (no IK selection needed)
                if (straight_line) {
                    if (keep_orientation) {
                        auto current_pose = arm.getCurrentPose().pose;
                        double base_tol = 0.05;
                        double tol_step = 0.05;
                        int max_attempts = 5;
                        bool planned = false;

                        for (int i = 0; i < max_attempts; i++) {
                            double tol = base_tol + (i * tol_step);
                            RCLCPP_INFO(logger, "Straight-line + orientation constraint attempt %d/%d, tolerance: %.3f rad",
                                        i + 1, max_attempts, tol);

                            moveit_msgs::msg::OrientationConstraint oc;
                            oc.header.frame_id = BASE_FRAME;
                            oc.link_name = EE_LINK;
                            oc.orientation = current_pose.orientation;
                            oc.absolute_x_axis_tolerance = tol;
                            oc.absolute_y_axis_tolerance = tol;
                            oc.absolute_z_axis_tolerance = tol;
                            oc.weight = 1.0;

                            moveit_msgs::msg::Constraints path_constraints;
                            path_constraints.orientation_constraints.push_back(oc);
                            arm.setPathConstraints(path_constraints);

                            std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                            moveit_msgs::msg::RobotTrajectory trajectory;
                            double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
                            if (fraction >= 0.99) {
                                arm.execute(trajectory);
                                planned = true;
                                break;
                            }
                        }
                        arm.clearPathConstraints();

                        response->response = json({
                            {"success", planned},
                            {"message", planned ? "Straight-line motion succeeded (orientation constrained)"
                                                : "Straight-line planning failed with orientation constraint"}
                        }).dump();
                    } else {
                        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                        moveit_msgs::msg::RobotTrajectory trajectory;
                        double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

                        if (fraction >= 0.99) {
                            arm.execute(trajectory);
                            response->response = json({{"success", true},
                                {"message", "Straight-line motion succeeded"}}).dump();
                        } else {
                            response->response = json({{"success", false},
                                {"message", "Straight-line planning achieved " +
                                            std::to_string(int(fraction * 100)) + "%"}}).dump();
                        }
                    }
                    return;
                }

                // --- Free-path mode: solve IK ourselves, pick closest, plan in joint space ---

                // 1. Get current joint state
                auto current_state = arm.getCurrentState(5.0);
                if (!current_state) {
                    response->response = json({{"success", false},
                        {"message", "Failed to get current robot state"}}).dump();
                    return;
                }
                const auto* jmg = current_state->getJointModelGroup(PLANNING_GROUP_ARM);
                std::vector<double> current_joints;
                current_state->copyJointGroupPositions(jmg, current_joints);

                // 2. Try IK multiple times with different seeds, keep closest solution
                const int IK_ATTEMPTS = 50;
                std::vector<double> best_solution;
                double best_distance = std::numeric_limits<double>::max();
                int solutions_found = 0;

                for (int i = 0; i < IK_ATTEMPTS; i++) {
                    // First attempt: seed with current joints. Rest: add random noise.
                    std::vector<double> seed = current_joints;
                    if (i > 0) {
                        for (auto& s : seed) {
                            s += ((double)rand() / RAND_MAX - 0.5) * 1.0;
                        }
                    }
                    current_state->setJointGroupPositions(jmg, seed);

                    if (current_state->setFromIK(jmg, target, 0.1)) {
                        std::vector<double> solution;
                        current_state->copyJointGroupPositions(jmg, solution);

                        // Calculate joint-space distance from current
                        double dist = 0.0;
                        for (size_t j = 0; j < solution.size(); j++) {
                            double d = solution[j] - current_joints[j];
                            dist += d * d;
                        }
                        dist = std::sqrt(dist);
                        solutions_found++;

                        if (dist < best_distance) {
                            best_distance = dist;
                            best_solution = solution;
                        }
                    }
                }

                if (best_solution.empty()) {
                    RCLCPP_ERROR(logger, "IK failed: no solution found in %d attempts", IK_ATTEMPTS);
                    response->response = json({{"success", false},
                        {"message", "No IK solution found for target pose"}}).dump();
                    return;
                }

                RCLCPP_INFO(logger, "IK: found %d solutions, best distance: %.4f rad", solutions_found, best_distance);
                RCLCPP_INFO(logger, "Best IK solution: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg",
                            best_solution[0] * 180.0 / M_PI, best_solution[1] * 180.0 / M_PI,
                            best_solution[2] * 180.0 / M_PI, best_solution[3] * 180.0 / M_PI,
                            best_solution[4] * 180.0 / M_PI, best_solution[5] * 180.0 / M_PI);

                // 3. Set orientation path constraint if requested
                if (keep_orientation) {
                    auto current_pose = arm.getCurrentPose().pose;
                    double base_tol = 0.05;
                    double tol_step = 0.05;
                    int max_attempts = 5;
                    bool planned = false;

                    for (int i = 0; i < max_attempts; i++) {
                        double tol = base_tol + (i * tol_step);
                        RCLCPP_INFO(logger, "Orientation constraint attempt %d/%d, tolerance: %.3f rad",
                                    i + 1, max_attempts, tol);

                        moveit_msgs::msg::OrientationConstraint oc;
                        oc.header.frame_id = BASE_FRAME;
                        oc.link_name = EE_LINK;
                        oc.orientation = current_pose.orientation;
                        oc.absolute_x_axis_tolerance = tol;
                        oc.absolute_y_axis_tolerance = tol;
                        oc.absolute_z_axis_tolerance = tol;
                        oc.weight = 1.0;

                        moveit_msgs::msg::Constraints path_constraints;
                        path_constraints.orientation_constraints.push_back(oc);
                        arm.setPathConstraints(path_constraints);

                        arm.setJointValueTarget(best_solution);
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        if (success) {
                            arm.execute(plan);
                            planned = true;
                            break;
                        }
                    }
                    arm.clearPathConstraints();

                    response->response = json({
                        {"success", planned},
                        {"message", planned ? "Motion succeeded (orientation constrained)"
                                            : "Planning failed with orientation constraint"}
                    }).dump();
                } else {
                    // 4. Plan in joint space to the best IK solution
                    arm.setJointValueTarget(best_solution);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (success) {
                        arm.execute(plan);
                        response->response = json({{"success", true},
                            {"message", "Cartesian motion succeeded (IK solutions: " +
                                        std::to_string(solutions_found) + ", distance: " +
                                        std::to_string(best_distance).substr(0, 5) + " rad)"}}).dump();
                    } else {
                        response->response = json({{"success", false},
                            {"message", "Planning failed (IK succeeded but path planning failed)"}}).dump();
                    }
                }

            } catch (const std::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    // ===================== Gripper Service (sim only) =====================
    // Pointer to keep the MoveGroupInterface alive for the lambda
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_ptr;
    rclcpp::Service<GripperCommand>::SharedPtr gripper_service;

    if (enable_gripper) {
        // Initialize the gripper interface with the dedicated moveit_node
        gripper_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node, PLANNING_GROUP_GRIPPER);
        RCLCPP_INFO(logger, "MoveGroupInterface ready for '%s'", PLANNING_GROUP_GRIPPER.c_str());

        gripper_service = service_node->create_service<GripperCommand>(
            "/gripper_command",
            [&gripper_ptr, &logger](
                const GripperCommand::Request::SharedPtr request,
                GripperCommand::Response::SharedPtr response)
            {
                try {
                    auto cmd = json::parse(request->command);
                    std::string action = cmd.at("action").get<std::string>();

                    if (action == "open") {
                        gripper_ptr->setNamedTarget("open");
                    } else if (action == "close") {
                        gripper_ptr->setNamedTarget("close");
                    } else {
                        response->response = json({{"success", false},
                            {"message", "Unknown action '" + action + "'. Use 'open' or 'close'."}}).dump();
                        return;
                    }

                    RCLCPP_INFO(logger, "Gripper command: %s", action.c_str());

                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    bool success = (gripper_ptr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (success) {
                        gripper_ptr->execute(plan);
                        response->response = json({{"success", true},
                            {"message", "Gripper " + action + " succeeded"}}).dump();
                    } else {
                        response->response = json({{"success", false},
                            {"message", "Gripper planning failed"}}).dump();
                    }

                } catch (const std::exception& e) {
                    response->response = json({{"success", false},
                        {"message", std::string("Error: ") + e.what()}}).dump();
                }
            }
        );
    } else {
        RCLCPP_INFO(logger, "Gripper service DISABLED (enable_gripper_service=false)");
    }

    RCLCPP_INFO(logger, "Services ready:");
    RCLCPP_INFO(logger, "  /joint_command     - Joint-space arm control");
    RCLCPP_INFO(logger, "  /cartesian_command - Cartesian arm control");
    if (enable_gripper) {
        RCLCPP_INFO(logger, "  /gripper_command   - Gripper open/close");
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}