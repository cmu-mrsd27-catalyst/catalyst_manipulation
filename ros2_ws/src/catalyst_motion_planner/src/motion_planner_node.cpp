#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <catalyst_interfaces/srv/json_command.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nlohmann/json.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm/wrapper/xarm_api.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <atomic>
#include <limits>
#include <cstdlib>

using json = nlohmann::json;
using JsonCommand = catalyst_interfaces::srv::JsonCommand;

const std::string PLANNING_GROUP_ARM = "xarm6";
const std::string PLANNING_GROUP_GRIPPER = "bio_gripper";
const std::string BASE_FRAME = "link_base";
const std::string EE_LINK = "link_tcp";
const std::string CONSTRAINT_LINK = "link_eef";  // last link in xarm6 group — use for path constraints
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
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");
    RCLCPP_INFO(logger, "###################################################################");

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
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(5);

    RCLCPP_INFO(logger, "MoveGroupInterface ready for '%s'", PLANNING_GROUP_ARM.c_str());

    // PlanningSceneInterface for applying ACM diffs (has its own executor, no deadlock)
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Fetch the full ACM at startup (outside any callback — no deadlock risk).
    // Retry until the ACM is populated (move_group may still be loading the SRDF).
    moveit_msgs::msg::AllowedCollisionMatrix cached_acm;
    {
        auto get_scene_client = service_node->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene");
        if (get_scene_client->wait_for_service(std::chrono::seconds(10))) {
            for (int attempt = 0; attempt < 10; ++attempt) {
                auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
                get_req->components.components = 128;  // ALLOWED_COLLISION_MATRIX
                auto future = get_scene_client->async_send_request(get_req);
                if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                    auto acm = future.get()->scene.allowed_collision_matrix;
                    if (!acm.entry_names.empty()) {
                        cached_acm = acm;
                        RCLCPP_INFO(logger, "Cached ACM with %zu entries", cached_acm.entry_names.size());
                        break;
                    }
                    RCLCPP_INFO(logger, "ACM empty on attempt %d, retrying in 1s...", attempt + 1);
                } else {
                    RCLCPP_WARN(logger, "ACM fetch timed out on attempt %d", attempt + 1);
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (cached_acm.entry_names.empty()) {
                RCLCPP_WARN(logger, "Failed to get populated ACM after retries — octomap toggle may not work");
            }
        } else {
            RCLCPP_WARN(logger, "get_planning_scene not available — octomap toggle may not work");
        }
    }

    // ===================== Set Octomap Enabled Service =====================
    auto set_octomap_service = service_node->create_service<std_srvs::srv::SetBool>(
        "/set_octomap_enabled",
        [&planning_scene_interface, &cached_acm, &logger](
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response)
        {
            if (cached_acm.entry_names.empty()) {
                RCLCPP_ERROR(logger, "No cached ACM available — cannot toggle octomap");
                response->success = false;
                response->message = "No cached ACM";
                return;
            }

            // Update <octomap> default entry in the cached ACM
            bool found = false;
            for (size_t i = 0; i < cached_acm.default_entry_names.size(); ++i) {
                if (cached_acm.default_entry_names[i] == "<octomap>") {
                    cached_acm.default_entry_values[i] = !request->data;
                    found = true;
                    break;
                }
            }
            if (!found) {
                cached_acm.default_entry_names.push_back("<octomap>");
                cached_acm.default_entry_values.push_back(!request->data);
            }

            // Apply the full ACM (entry_names is populated so the diff guard passes)
            moveit_msgs::msg::PlanningScene ps;
            ps.is_diff = true;
            ps.allowed_collision_matrix = cached_acm;

            if (!request->data) {
                RCLCPP_INFO(logger, "Octomap DISABLED for planning (collisions allowed)");
            } else {
                RCLCPP_INFO(logger, "Octomap ENABLED for planning (collisions checked)");
            }

            bool ok = planning_scene_interface.applyPlanningScene(ps);
            response->success = ok;
            response->message = ok
                ? (request->data ? "Octomap enabled" : "Octomap disabled")
                : "applyPlanningScene failed";
        }
    );

    // Compute the "flat" EEF orientation at home (all joints zero) for keep_orientation constraint.
    // This orientation has the gripper pointing straight down — flat w.r.t. the table.
    // keep_orientation will constrain X/Y tilt to stay near this, while allowing free Z rotation.
    geometry_msgs::msg::Quaternion flat_orientation;
    {
        auto state = arm.getCurrentState(5.0);
        if (state) {
            const auto* jmg = state->getJointModelGroup(PLANNING_GROUP_ARM);
            if (state->setToDefaultValues(jmg, "home")) {
                state->update();
                auto tf = state->getGlobalLinkTransform(CONSTRAINT_LINK);
                Eigen::Quaterniond q(tf.rotation());
                flat_orientation.x = q.x();
                flat_orientation.y = q.y();
                flat_orientation.z = q.z();
                flat_orientation.w = q.w();
                RCLCPP_INFO(logger, "Flat (home) orientation for %s: quat=(%.4f, %.4f, %.4f, %.4f)",
                            CONSTRAINT_LINK.c_str(), q.x(), q.y(), q.z(), q.w());
            } else {
                RCLCPP_WARN(logger, "Could not resolve home pose — defaulting flat_orientation to identity");
                flat_orientation.x = 0.0; flat_orientation.y = 0.0;
                flat_orientation.z = 0.0; flat_orientation.w = 1.0;
            }
        } else {
            RCLCPP_WARN(logger, "Could not get robot state — defaulting flat_orientation to identity");
            flat_orientation.x = 0.0; flat_orientation.y = 0.0;
            flat_orientation.z = 0.0; flat_orientation.w = 1.0;
        }
    }

    // ===================== Joint Service =====================
    auto joint_service = service_node->create_service<JsonCommand>(
        "/joint_command",
        [&arm, &logger](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
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

                // Plan and execute with retry on validation failure
                const int MAX_RETRIES = 5;
                bool succeeded = false;
                for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
                    auto move_result = arm.move();
                    if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        response->response = json({{"success", true},
                            {"message", "Joint motion succeeded"}}).dump();
                        succeeded = true;
                        break;
                    }
                    RCLCPP_WARN(logger, "Joint move attempt %d/%d failed (code %d), retrying...",
                                attempt, MAX_RETRIES, move_result.val);
                }
                if (!succeeded) {
                    response->response = json({{"success", false},
                        {"message", "Joint motion failed after " + std::to_string(MAX_RETRIES) + " attempts"}}).dump();
                }

            } catch (const std::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    // ===================== Cartesian Service =====================
    auto cartesian_service = service_node->create_service<JsonCommand>(
        "/cartesian_command",
        [&arm, &logger, &flat_orientation](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
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
                        double base_tol = 0.05;
                        double tol_step = 0.05;
                        int max_attempts = 5;
                        bool planned = false;

                        for (int i = 0; i < max_attempts; i++) {
                            double tol = base_tol + (i * tol_step);
                            RCLCPP_INFO(logger, "Straight-line + orientation constraint attempt %d/%d, xy_tol: %.3f rad",
                                        i + 1, max_attempts, tol);

                            // Keep gripper flat: free rotation around link_eef X, constrain Y and Z
                            moveit_msgs::msg::OrientationConstraint oc;
                            oc.header.frame_id = BASE_FRAME;
                            oc.link_name = CONSTRAINT_LINK;
                            oc.orientation = flat_orientation;
                            oc.absolute_x_axis_tolerance = M_PI;
                            oc.absolute_y_axis_tolerance = tol;
                            oc.absolute_z_axis_tolerance = tol;
                            oc.parameterization = 1;  // ROTATION_VECTOR
                            oc.weight = 1.0;

                            moveit_msgs::msg::Constraints path_constraints;
                            path_constraints.orientation_constraints.push_back(oc);
                            arm.setPathConstraints(path_constraints);

                            std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                            moveit_msgs::msg::RobotTrajectory trajectory;
                            double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);
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
                        double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

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

                // 2. Solve IK with multiple seeds, pick solution closest to current joints.
                //    This avoids unnecessary joint 1 flips when multiple IK configs exist.
                const int IK_ATTEMPTS = 20;
                double best_distance = std::numeric_limits<double>::max();
                std::vector<double> best_solution;
                int solutions_found = 0;

                for (int i = 0; i < IK_ATTEMPTS; i++) {
                    std::vector<double> seed = current_joints;
                    if (i > 0) {
                        for (size_t j = 0; j < seed.size(); j++) {
                            // Larger perturbation on joint 1 to explore different base configs
                            double range = (j == 0) ? 2.0 : 0.8;
                            seed[j] += ((double)rand() / RAND_MAX - 0.5) * range;
                        }
                    }
                    current_state->setJointGroupPositions(jmg, seed);

                    if (current_state->setFromIK(jmg, target, "link_tcp", 0.1)) {
                        std::vector<double> solution;
                        current_state->copyJointGroupPositions(jmg, solution);
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

                if (solutions_found == 0) {
                    RCLCPP_ERROR(logger, "IK failed: no solution found in %d attempts", IK_ATTEMPTS);
                    response->response = json({{"success", false},
                        {"message", "No IK solution found for target pose"}}).dump();
                    return;
                }

                RCLCPP_INFO(logger, "IK: %d solutions found, best joint dist: %.3f rad", solutions_found, best_distance);
                RCLCPP_INFO(logger, "IK solution: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg",
                            best_solution[0] * 180.0 / M_PI, best_solution[1] * 180.0 / M_PI,
                            best_solution[2] * 180.0 / M_PI, best_solution[3] * 180.0 / M_PI,
                            best_solution[4] * 180.0 / M_PI, best_solution[5] * 180.0 / M_PI);

                // 3. Set orientation path constraint if requested
                if (keep_orientation) {
                    double base_tol = 0.05;
                    double tol_step = 0.05;
                    int max_attempts = 5;
                    bool planned = false;

                    for (int i = 0; i < max_attempts; i++) {
                        double tol = base_tol + (i * tol_step);
                        RCLCPP_INFO(logger, "Orientation constraint attempt %d/%d, xy_tol: %.3f rad",
                                    i + 1, max_attempts, tol);

                        // Keep gripper flat: free rotation around link_eef X, constrain Y and Z
                        moveit_msgs::msg::OrientationConstraint oc;
                        oc.header.frame_id = BASE_FRAME;
                        oc.link_name = CONSTRAINT_LINK;
                        oc.orientation = flat_orientation;
                        oc.absolute_x_axis_tolerance = M_PI;
                        oc.absolute_y_axis_tolerance = tol;
                        oc.absolute_z_axis_tolerance = tol;
                        oc.parameterization = 1;  // ROTATION_VECTOR
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
                    // 4. Plan in joint space to the best IK solution, retry on validation failure
                    arm.setJointValueTarget(best_solution);
                    const int MAX_CART_RETRIES = 5;
                    bool cart_succeeded = false;
                    for (int attempt = 1; attempt <= MAX_CART_RETRIES; attempt++) {
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        bool plan_ok = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        if (plan_ok) {
                            auto exec_res = arm.execute(plan);
                            if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
                                response->response = json({{"success", true},
                                    {"message", "Cartesian motion succeeded"}}).dump();
                                cart_succeeded = true;
                                break;
                            }
                        }
                        RCLCPP_WARN(logger, "Cartesian move attempt %d/%d failed, retrying...",
                                    attempt, MAX_CART_RETRIES);
                    }
                    if (!cart_succeeded) {
                        response->response = json({{"success", false},
                            {"message", "Cartesian motion failed after " + std::to_string(MAX_CART_RETRIES) + " attempts"}}).dump();
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
    rclcpp::Service<JsonCommand>::SharedPtr gripper_service;

    if (enable_gripper) {
        // Initialize the gripper interface with the dedicated moveit_node
        gripper_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node, PLANNING_GROUP_GRIPPER);
        RCLCPP_INFO(logger, "MoveGroupInterface ready for '%s'", PLANNING_GROUP_GRIPPER.c_str());

        gripper_service = service_node->create_service<JsonCommand>(
            "/gripper_command",
            [&gripper_ptr, &logger](
                const JsonCommand::Request::SharedPtr request,
                JsonCommand::Response::SharedPtr response)
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

    // ===================== Guide Mode Service =====================
    bool enable_guide_mode = service_node->get_parameter_or("enable_guide_mode", rclcpp::Parameter("enable_guide_mode", false)).as_bool();
    std::string robot_ip = service_node->get_parameter_or("robot_ip", rclcpp::Parameter("robot_ip", "192.168.1.212")).as_string();

    XArmAPI* xarm_ptr = nullptr;
    std::atomic<bool> is_teach_mode{false};
    rclcpp::Service<JsonCommand>::SharedPtr guide_mode_service;

    if (enable_guide_mode) {
        xarm_ptr = new XArmAPI(robot_ip);
        if (xarm_ptr->error_code != 0) {
            RCLCPP_ERROR(logger, "Failed to connect xArm SDK to %s (error=%d)", robot_ip.c_str(), xarm_ptr->error_code);
            delete xarm_ptr;
            xarm_ptr = nullptr;
        } else {
            RCLCPP_INFO(logger, "xArm SDK connected to %s for guide mode", robot_ip.c_str());
        }
    }

    // Max time to wait for arm mode/state confirmation
    const int GUIDE_MODE_CONFIRM_TIMEOUT_MS = 5000;
    const int GUIDE_MODE_POLL_INTERVAL_MS = 100;

    if (enable_guide_mode && xarm_ptr) {
        guide_mode_service = service_node->create_service<JsonCommand>(
            "/guide_mode",
            [&xarm_ptr, &is_teach_mode, &logger,
             GUIDE_MODE_CONFIRM_TIMEOUT_MS, GUIDE_MODE_POLL_INTERVAL_MS](
                const JsonCommand::Request::SharedPtr request,
                JsonCommand::Response::SharedPtr response)
            {
                // Helper: poll until arm reaches expected mode+state, or timeout
                auto wait_for_mode_state = [&](int expected_mode, int expected_state,
                                               int timeout_ms, int poll_ms) -> bool {
                    int elapsed = 0;
                    while (elapsed < timeout_ms) {
                        if (xarm_ptr->mode == expected_mode && xarm_ptr->state == expected_state) {
                            return true;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
                        elapsed += poll_ms;
                    }
                    return (xarm_ptr->mode == expected_mode && xarm_ptr->state == expected_state);
                };

                try {
                    auto cmd = json::parse(request->command);
                    std::string action = cmd.at("action").get<std::string>();

                    if (action == "enable") {
                        // Set teach sensitivity (1–5, default 3)
                        int sensitivity = cmd.value("sensitivity", 3);
                        sensitivity = std::clamp(sensitivity, 1, 5);
                        xarm_ptr->set_teach_sensitivity(sensitivity);
                        RCLCPP_INFO(logger, "Teach sensitivity set to %d", sensitivity);

                        // Step 1: STOP — HW plugin detects state>2, auto-deactivates controllers
                        int ret = xarm_ptr->set_state(4);  // STOP
                        if (ret != 0) {
                            response->response = json({
                                {"success", false},
                                {"message", "set_state(STOP) failed, ret=" + std::to_string(ret)},
                                {"mode", xarm_ptr->mode},
                                {"state", xarm_ptr->state}
                            }).dump();
                            return;
                        }

                        // Step 2: Wait for HW plugin to finish deactivation and stop writing
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Step 3: Set teach mode directly — do NOT call clean_error/
                        // motion_enable here, they reset mode to 0 (POSE) and the HW
                        // plugin might briefly see "ready" and auto-reactivate controllers.
                        xarm_ptr->set_mode(2);   // TEACH
                        xarm_ptr->set_state(0);  // START

                        // Step 4: Check mode=2 (state doesn't need to be 0 — state=2 is also valid in teach)
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        bool confirmed = (xarm_ptr->mode == 2);

                        if (!confirmed) {
                            // Retry: the HW plugin may have reset mode to 0. Clear errors
                            // and re-enable motion before retrying (matches guide_mode.py).
                            RCLCPP_WARN(logger, "Teach mode did not take effect (mode=%d, state=%d), "
                                        "retrying with clean_error...", xarm_ptr->mode, xarm_ptr->state);
                            xarm_ptr->clean_error();
                            xarm_ptr->clean_warn();
                            xarm_ptr->motion_enable(true);
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                            xarm_ptr->set_mode(2);   // TEACH
                            xarm_ptr->set_state(0);  // START

                            // Poll until mode=2 or timeout
                            std::this_thread::sleep_for(std::chrono::milliseconds(300));
                            confirmed = (xarm_ptr->mode == 2);
                        }

                        if (!confirmed) {
                            RCLCPP_ERROR(logger, "Guide mode enable FAILED after retry (mode=%d, state=%d)",
                                         xarm_ptr->mode, xarm_ptr->state);
                            response->response = json({
                                {"success", false},
                                {"message", "Timeout waiting for teach mode confirmation after retry"},
                                {"mode", xarm_ptr->mode},
                                {"state", xarm_ptr->state}
                            }).dump();
                            return;
                        }

                        is_teach_mode.store(true);
                        RCLCPP_INFO(logger, "Guide mode ENABLED (teach mode confirmed: mode=%d, state=%d)",
                                    xarm_ptr->mode, xarm_ptr->state);
                        response->response = json({
                            {"success", true},
                            {"message", "Guide mode enabled"},
                            {"mode", xarm_ptr->mode},
                            {"state", xarm_ptr->state}
                        }).dump();

                    } else if (action == "disable") {
                        // Full reset sequence (matches guide_mode.py disable_teach_mode):
                        // clean_error + clean_warn + motion_enable + set_mode(SERVO) + set_state(START)
                        xarm_ptr->clean_error();
                        xarm_ptr->clean_warn();
                        xarm_ptr->motion_enable(true);
                        xarm_ptr->set_mode(1);   // SERVO
                        xarm_ptr->set_state(0);  // START

                        // Wait for HW plugin to detect arm is ready and auto-reactivate
                        // all controllers (joint_state_broadcaster + xarm6_traj_controller).
                        RCLCPP_INFO(logger, "Servo mode set, waiting for HW plugin to reactivate controllers...");
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Verify mode/state
                        bool confirmed = wait_for_mode_state(1, 0,
                            GUIDE_MODE_CONFIRM_TIMEOUT_MS, GUIDE_MODE_POLL_INTERVAL_MS);

                        if (!confirmed) {
                            RCLCPP_WARN(logger, "Guide mode disable: arm did not confirm mode=1/state=0 "
                                        "within timeout (got mode=%d, state=%d)", xarm_ptr->mode, xarm_ptr->state);
                            response->response = json({
                                {"success", false},
                                {"message", "Timeout waiting for servo mode confirmation"},
                                {"mode", xarm_ptr->mode},
                                {"state", xarm_ptr->state}
                            }).dump();
                            return;
                        }

                        is_teach_mode.store(false);
                        RCLCPP_INFO(logger, "Guide mode DISABLED (servo mode confirmed: mode=%d, state=%d)",
                                    xarm_ptr->mode, xarm_ptr->state);
                        response->response = json({
                            {"success", true},
                            {"message", "Guide mode disabled"},
                            {"mode", xarm_ptr->mode},
                            {"state", xarm_ptr->state}
                        }).dump();

                    } else if (action == "status") {
                        response->response = json({
                            {"success", true},
                            {"message", "Current arm status"},
                            {"mode", xarm_ptr->mode},
                            {"state", xarm_ptr->state}
                        }).dump();

                    } else {
                        response->response = json({
                            {"success", false},
                            {"message", "Unknown action '" + action + "'. Use 'enable', 'disable', or 'status'."}
                        }).dump();
                    }

                } catch (const std::exception& e) {
                    response->response = json({{"success", false},
                        {"message", std::string("Error: ") + e.what()}}).dump();
                }
            }
        );
    } else if (enable_guide_mode) {
        RCLCPP_WARN(logger, "Guide mode requested but xArm SDK connection failed — service not created");
    } else {
        RCLCPP_INFO(logger, "Guide mode service DISABLED (enable_guide_mode=false)");
    }

    RCLCPP_INFO(logger, "Services ready:");
    RCLCPP_INFO(logger, "  /joint_command         - Joint-space arm control");
    RCLCPP_INFO(logger, "  /cartesian_command     - Cartesian arm control");
    RCLCPP_INFO(logger, "  /set_octomap_enabled   - Enable/disable octomap for planning");
    if (enable_gripper) {
        RCLCPP_INFO(logger, "  /gripper_command   - Gripper open/close");
    }
    if (enable_guide_mode && xarm_ptr) {
        RCLCPP_INFO(logger, "  /guide_mode        - Toggle teach/guide mode");
    }

    spin_thread.join();

    // Auto-disable teach mode on shutdown
    if (xarm_ptr && is_teach_mode.load()) {
        RCLCPP_INFO(logger, "Shutting down: auto-disabling teach mode...");
        xarm_ptr->clean_error();
        xarm_ptr->clean_warn();
        xarm_ptr->motion_enable(true);
        xarm_ptr->set_mode(1);
        xarm_ptr->set_state(0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    if (xarm_ptr) {
        xarm_ptr->disconnect();
        delete xarm_ptr;
    }

    rclcpp::shutdown();
    return 0;
}
