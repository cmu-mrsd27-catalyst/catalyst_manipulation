#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <catalyst_interfaces/srv/json_command.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <nlohmann/json.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm/wrapper/xarm_api.h>
#include <thread>
#include <chrono>
#include <future>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <atomic>
#include <limits>
#include <optional>
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

const char * const kArmTrajController = "xarm6_traj_controller";
const char * const kAdmittanceController = "admittance_controller";
const char * const kJointStateBroadcaster = "joint_state_broadcaster";
const char * const kFtBroadcaster = "force_torque_sensor_broadcaster";

/** UFactory xArm HW reports state 5 (CONFIG_CHANGED) after SDK calls like set_collision_sensitivity;
 *  the HW plugin then deactivates traj + broadcasters. MoveIt still plans; execute rejects goals.
 *  Reactivate traj and (if present) joint_state + FT broadcasters after CM recovers from overruns. */
static void ensure_xarm6_traj_controller_active(
    const rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr & list_cli,
    const rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr & switch_cli,
    const rclcpp::Logger & logger)
{
    if (!list_cli->service_is_ready()) {
        if (!list_cli->wait_for_service(std::chrono::seconds(2))) {
            return;
        }
    }

    controller_manager_msgs::srv::ListControllers::Response::SharedPtr list_resp;
    bool list_ok = false;
    for (int attempt = 0; attempt < 3 && rclcpp::ok(); ++attempt) {
        if (attempt > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto list_future = list_cli->async_send_request(list_req);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        bool got = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (list_future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
                list_resp = list_future.get();
                got = true;
                list_ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (got) {
            break;
        }
        RCLCPP_WARN(
            logger,
            "ensure_xarm6_traj: list_controllers timed out (attempt %d/3) — controller_manager slow or client/executor deadlock",
            attempt + 1);
    }
    if (!list_ok || !list_resp) {
        return;
    }
    bool traj_active = false;
    bool js_active = false;
    bool ft_in_list = false;
    bool ft_active = false;
    bool admittance_active = false;
    for (const auto & c : list_resp->controller) {
        if (c.name == kArmTrajController && c.state == "active") {
            traj_active = true;
        }
        if (c.name == kJointStateBroadcaster && c.state == "active") {
            js_active = true;
        }
        if (c.name == kFtBroadcaster) {
            ft_in_list = true;
            ft_active = (c.state == "active");
        }
        if (c.name == kAdmittanceController && c.state == "active") {
            admittance_active = true;
        }
    }
    const bool need_ft = ft_in_list && !ft_active;
    if (traj_active && js_active && !need_ft) {
        return;
    }

    RCLCPP_WARN(
        logger,
        "Arm stack not fully active (traj=%d joint_state_broadcaster=%d ft_broadcaster=%d) — switching so MoveIt can execute",
        traj_active ? 1 : 0,
        js_active ? 1 : 0,
        (ft_in_list && ft_active) ? 1 : (ft_in_list ? 0 : -1));

    if (!switch_cli->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger, "controller_manager/switch_controller not available");
        return;
    }

    auto sw_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (admittance_active) {
        sw_req->deactivate_controllers.push_back(kAdmittanceController);
    }
    if (!traj_active) {
        sw_req->activate_controllers.push_back(kArmTrajController);
    }
    if (!js_active) {
        sw_req->activate_controllers.push_back(kJointStateBroadcaster);
    }
    if (need_ft) {
        sw_req->activate_controllers.push_back(kFtBroadcaster);
    }
    sw_req->strictness =
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    sw_req->activate_asap = true;

    auto sw_future = switch_cli->async_send_request(sw_req);
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (sw_future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    if (sw_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        RCLCPP_WARN(logger, "ensure_xarm6_traj: switch_controller timed out");
        return;
    }
    auto sw_resp = sw_future.get();
    if (sw_resp->ok) {
        RCLCPP_INFO(logger, "Re-activated arm controllers for trajectory execution");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else {
        RCLCPP_WARN(logger, "switch_controller returned ok=false");
    }
}

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
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(3);

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

    auto get_planning_scene_client =
        service_node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    // ===================== Set Octomap Enabled Service =====================
    auto set_octomap_service = service_node->create_service<std_srvs::srv::SetBool>(
        "/set_octomap_enabled",
        [&planning_scene_interface, &cached_acm, &logger, get_planning_scene_client](
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response)
        {
            moveit_msgs::msg::AllowedCollisionMatrix acm;

            if (get_planning_scene_client->wait_for_service(std::chrono::seconds(2))) {
                auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
                get_req->components.components = 128;  // ALLOWED_COLLISION_MATRIX
                auto future = get_planning_scene_client->async_send_request(get_req);
                if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                    auto scene_resp = future.get();
                    if (scene_resp &&
                        !scene_resp->scene.allowed_collision_matrix.entry_names.empty()) {
                        acm = scene_resp->scene.allowed_collision_matrix;
                    }
                }
            }

            if (acm.entry_names.empty()) {
                if (cached_acm.entry_names.empty()) {
                    RCLCPP_ERROR(logger, "No ACM available — cannot toggle octomap");
                    response->success = false;
                    response->message = "No ACM available";
                    return;
                }
                acm = cached_acm;
            }

            // Merge <octomap> default entry into the live ACM (preserves e.g. workspace_box entries).
            bool found = false;
            for (size_t i = 0; i < acm.default_entry_names.size(); ++i) {
                if (acm.default_entry_names[i] == "<octomap>") {
                    acm.default_entry_values[i] = !request->data;
                    found = true;
                    break;
                }
            }
            if (!found) {
                acm.default_entry_names.push_back("<octomap>");
                acm.default_entry_values.push_back(!request->data);
            }

            moveit_msgs::msg::PlanningScene ps;
            ps.is_diff = true;
            ps.allowed_collision_matrix = acm;

            if (!request->data) {
                RCLCPP_INFO(logger, "Octomap DISABLED for planning (collisions allowed)");
            } else {
                RCLCPP_INFO(logger, "Octomap ENABLED for planning (collisions checked)");
            }

            bool ok = planning_scene_interface.applyPlanningScene(ps);
            if (ok) {
                cached_acm = acm;
            }
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

    // Flag set by /stop_motion — checked in all motion retry loops
    std::atomic<bool> stop_requested{false};

    // ── EEF bounds (PositionConstraint on CONSTRAINT_LINK = link_eef) ──
    struct EefBounds {
        std::mutex mutex;
        bool active = false;
        double x_min = 0, x_max = 0;
        double z_min = 0, z_max = 0;
        // Y is unconstrained — use large range
    };
    EefBounds eef_bounds;

    // Helper: apply or clear EEF bounds as path constraint
    auto apply_eef_bounds = [&arm, &eef_bounds, &logger]() {
        std::lock_guard<std::mutex> lock(eef_bounds.mutex);
        if (!eef_bounds.active) {
            // Don't clear here — keep_orientation sets its own constraints
            return;
        }
        // Build a box constraint for link_eef (last link in xarm6 group) in link_base frame
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = BASE_FRAME;
        pc.link_name = CONSTRAINT_LINK;
        pc.weight = 1.0;

        // Box primitive
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {
            eef_bounds.x_max - eef_bounds.x_min,  // X size
            2.0,                                    // Y size (unconstrained)
            eef_bounds.z_max - eef_bounds.z_min,   // Z size
        };

        // Box center pose
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = (eef_bounds.x_min + eef_bounds.x_max) / 2.0;
        box_pose.position.y = 0.0;  // centered at origin Y
        box_pose.position.z = (eef_bounds.z_min + eef_bounds.z_max) / 2.0;
        box_pose.orientation.w = 1.0;

        pc.constraint_region.primitives.push_back(box);
        pc.constraint_region.primitive_poses.push_back(box_pose);

        moveit_msgs::msg::Constraints constraints;
        constraints.position_constraints.push_back(pc);
        arm.setPathConstraints(constraints);

        RCLCPP_INFO(logger, "EEF bounds applied: X[%.3f, %.3f] Z[%.3f, %.3f]",
                    eef_bounds.x_min, eef_bounds.x_max,
                    eef_bounds.z_min, eef_bounds.z_max);
    };

    auto clear_eef_bounds = [&arm]() {
        arm.clearPathConstraints();
    };

    // Helper: build a PositionConstraint if bounds are active (caller must hold lock)
    auto build_position_constraint = [&eef_bounds]() -> std::optional<moveit_msgs::msg::PositionConstraint> {
        if (!eef_bounds.active) return std::nullopt;

        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = BASE_FRAME;
        pc.link_name = CONSTRAINT_LINK;
        pc.weight = 1.0;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {
            eef_bounds.x_max - eef_bounds.x_min,
            2.0,
            eef_bounds.z_max - eef_bounds.z_min,
        };

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = (eef_bounds.x_min + eef_bounds.x_max) / 2.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = (eef_bounds.z_min + eef_bounds.z_max) / 2.0;
        box_pose.orientation.w = 1.0;

        pc.constraint_region.primitives.push_back(box);
        pc.constraint_region.primitive_poses.push_back(box_pose);
        return pc;
    };

    // Helper: apply orientation + optional EEF bounds as combined path constraints
    auto apply_combined_constraints = [&arm, &eef_bounds, &build_position_constraint](
            const moveit_msgs::msg::Constraints& base_constraints) {
        std::lock_guard<std::mutex> lock(eef_bounds.mutex);
        auto pc = build_position_constraint();
        if (pc) {
            auto combined = base_constraints;
            combined.position_constraints.push_back(*pc);
            arm.setPathConstraints(combined);
        } else {
            arm.setPathConstraints(base_constraints);
        }
    };

    /* Clients must NOT use the default callback group: /joint_command and
     * /cartesian_command block on wait_for(future). Service callbacks and client
     * responses in the same mutually exclusive group deadlock — futures never complete. */
    auto cm_clients_cb_group =
        service_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto list_ctrl_client =
        service_node->create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers",
            rclcpp::ServicesQoS(),
            cm_clients_cb_group);
    auto switch_ctrl_client =
        service_node->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller",
            rclcpp::ServicesQoS(),
            cm_clients_cb_group);

    // ===================== Joint Service =====================
    auto joint_service = service_node->create_service<JsonCommand>(
        "/joint_command",
        [&arm, &logger, &stop_requested, &apply_eef_bounds, &clear_eef_bounds,
         list_ctrl_client, switch_ctrl_client](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
        {
            stop_requested.store(false);  // clear flag at start of new motion
            ensure_xarm6_traj_controller_active(list_ctrl_client, switch_ctrl_client, logger);
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

                // Apply EEF bounds if active
                apply_eef_bounds();

                // Plan and execute with retry on validation failure
                const int MAX_RETRIES = 5;
                bool succeeded = false;
                for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
                    if (stop_requested.load()) {
                        clear_eef_bounds();
                        response->response = json({{"success", false},
                            {"message", "Motion stopped by /stop_motion"}}).dump();
                        return;
                    }
                    auto move_result = arm.move();
                    if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        response->response = json({{"success", true},
                            {"message", "Joint motion succeeded"}}).dump();
                        succeeded = true;
                        break;
                    }
                    if (stop_requested.load()) {
                        response->response = json({{"success", false},
                            {"message", "Motion stopped by /stop_motion"}}).dump();
                        return;
                    }
                    RCLCPP_WARN(logger, "Joint move attempt %d/%d failed (code %d), retrying...",
                                attempt, MAX_RETRIES, move_result.val);
                }
                if (!succeeded) {
                    response->response = json({{"success", false},
                        {"message", "Joint motion failed after " + std::to_string(MAX_RETRIES) + " attempts"}}).dump();
                }
                clear_eef_bounds();

            } catch (const std::exception& e) {
                clear_eef_bounds();
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    // ===================== Cartesian Service =====================
    auto cartesian_service = service_node->create_service<JsonCommand>(
        "/cartesian_command",
        [&arm, &logger, &flat_orientation, &stop_requested,
         &apply_eef_bounds, &clear_eef_bounds, &apply_combined_constraints,
         list_ctrl_client, switch_ctrl_client](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
        {
            stop_requested.store(false);  // clear flag at start of new motion
            ensure_xarm6_traj_controller_active(list_ctrl_client, switch_ctrl_client, logger);
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
                            if (stop_requested.load()) break;
                            double tol = base_tol + (i * tol_step);
                            RCLCPP_INFO(logger, "Straight-line + orientation constraint attempt %d/%d, xy_tol: %.3f rad",
                                        i + 1, max_attempts, tol);

                            moveit_msgs::msg::OrientationConstraint oc;
                            oc.header.frame_id = BASE_FRAME;
                            oc.link_name = CONSTRAINT_LINK;
                            oc.orientation = flat_orientation;
                            oc.absolute_x_axis_tolerance = M_PI;
                            oc.absolute_y_axis_tolerance = tol;
                            oc.absolute_z_axis_tolerance = tol;
                            oc.parameterization = 1;
                            oc.weight = 1.0;

                            moveit_msgs::msg::Constraints path_constraints;
                            path_constraints.orientation_constraints.push_back(oc);
                            apply_combined_constraints(path_constraints);

                            std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                            moveit_msgs::msg::RobotTrajectory trajectory;
                            double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);
                            if (fraction >= 0.99) {
                                arm.execute(trajectory);
                                planned = !stop_requested.load();
                                break;
                            }
                        }
                        clear_eef_bounds();

                        response->response = json({
                            {"success", planned},
                            {"message", planned ? "Straight-line motion succeeded (orientation constrained)"
                                                : "Straight-line planning failed with orientation constraint"}
                        }).dump();
                    } else {
                        apply_eef_bounds();
                        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                        moveit_msgs::msg::RobotTrajectory trajectory;
                        double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

                        if (fraction >= 0.99) {
                            arm.execute(trajectory);
                            clear_eef_bounds();
                            response->response = json({{"success", true},
                                {"message", "Straight-line motion succeeded"}}).dump();
                        } else {
                            clear_eef_bounds();
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
                        if (stop_requested.load()) break;
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
                        apply_combined_constraints(path_constraints);

                        arm.setJointValueTarget(best_solution);
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        if (success) {
                            arm.execute(plan);
                            planned = true;
                            break;
                        }
                    }
                    clear_eef_bounds();

                    response->response = json({
                        {"success", planned},
                        {"message", planned ? "Motion succeeded (orientation constrained)"
                                            : "Planning failed with orientation constraint"}
                    }).dump();
                } else {
                    // 4. Plan in joint space to the best IK solution, retry on validation failure
                    apply_eef_bounds();
                    arm.setJointValueTarget(best_solution);
                    const int MAX_CART_RETRIES = 5;
                    bool cart_succeeded = false;
                    for (int attempt = 1; attempt <= MAX_CART_RETRIES; attempt++) {
                        if (stop_requested.load()) {
                            clear_eef_bounds();
                            response->response = json({{"success", false},
                                {"message", "Motion stopped by /stop_motion"}}).dump();
                            return;
                        }
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
                            if (stop_requested.load()) {
                                response->response = json({{"success", false},
                                    {"message", "Motion stopped by /stop_motion"}}).dump();
                                return;
                            }
                        }
                        RCLCPP_WARN(logger, "Cartesian move attempt %d/%d failed, retrying...",
                                    attempt, MAX_CART_RETRIES);
                    }
                    clear_eef_bounds();
                    if (!cart_succeeded) {
                        response->response = json({{"success", false},
                            {"message", "Cartesian motion failed after " + std::to_string(MAX_CART_RETRIES) + " attempts"}}).dump();
                    }
                }

            } catch (const std::exception& e) {
                clear_eef_bounds();
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
                        int sensitivity = cmd.value("sensitivity", 5);
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

    // ===================== Collision Sensitivity Service =====================
    rclcpp::Service<JsonCommand>::SharedPtr collision_sensitivity_service;
    if (xarm_ptr) {
        collision_sensitivity_service = service_node->create_service<JsonCommand>(
            "/set_collision_sensitivity",
            [&xarm_ptr, &logger, list_ctrl_client, switch_ctrl_client](
                const JsonCommand::Request::SharedPtr request,
                JsonCommand::Response::SharedPtr response)
            {
                try {
                    auto cmd = json::parse(request->command);
                    int level = cmd.at("level").get<int>();
                    level = std::clamp(level, 0, 5);
                    int ret = xarm_ptr->set_collision_sensitivity(level);
                    RCLCPP_INFO(logger, "Collision sensitivity set to %d (ret=%d)", level, ret);
                    if (ret == 0) {
                        /* UFactory HW often reports state 5 (CONFIG_CHANGED) and deactivates
                         * traj + broadcasters; controller_manager can stall ~1s. Brief pause
                         * then re-arm controllers before the next /cartesian_command. */
                        std::this_thread::sleep_for(std::chrono::milliseconds(800));
                        ensure_xarm6_traj_controller_active(list_ctrl_client, switch_ctrl_client, logger);
                    }
                    response->response = json({
                        {"success", ret == 0},
                        {"message", ret == 0 ? "Collision sensitivity set to " + std::to_string(level)
                                             : "Failed, ret=" + std::to_string(ret)}
                    }).dump();
                } catch (const std::exception& e) {
                    response->response = json({{"success", false},
                        {"message", std::string("Error: ") + e.what()}}).dump();
                }
            }
        );
    }

    // ===================== EEF Bounds Service =====================
    auto eef_bounds_service = service_node->create_service<JsonCommand>(
        "/set_eef_bounds",
        [&eef_bounds, &logger](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
        {
            try {
                auto cmd = json::parse(request->command);
                std::string action = cmd.at("action").get<std::string>();

                if (action == "set") {
                    std::lock_guard<std::mutex> lock(eef_bounds.mutex);
                    eef_bounds.x_min = cmd.at("x_min").get<double>();
                    eef_bounds.x_max = cmd.at("x_max").get<double>();
                    eef_bounds.z_min = cmd.at("z_min").get<double>();
                    eef_bounds.z_max = cmd.at("z_max").get<double>();
                    eef_bounds.active = true;
                    RCLCPP_INFO(logger, "EEF bounds SET: X[%.3f, %.3f] Z[%.3f, %.3f]",
                                eef_bounds.x_min, eef_bounds.x_max,
                                eef_bounds.z_min, eef_bounds.z_max);
                    response->response = json({{"success", true},
                        {"message", "EEF bounds active"}}).dump();

                } else if (action == "clear") {
                    std::lock_guard<std::mutex> lock(eef_bounds.mutex);
                    eef_bounds.active = false;
                    RCLCPP_INFO(logger, "EEF bounds CLEARED");
                    response->response = json({{"success", true},
                        {"message", "EEF bounds cleared"}}).dump();

                } else {
                    response->response = json({{"success", false},
                        {"message", "Unknown action. Use 'set' or 'clear'."}}).dump();
                }
            } catch (const std::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    // ===================== Stop Motion Service =====================
    // Must use a separate ReentrantCallbackGroup so it can execute
    // even while /cartesian_command or /joint_command is blocking in arm.execute().
    auto stop_cb_group = service_node->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    auto stop_motion_service = service_node->create_service<std_srvs::srv::Empty>(
        "/stop_motion",
        [&arm, &logger, &stop_requested](
            const std_srvs::srv::Empty::Request::SharedPtr /*request*/,
            std_srvs::srv::Empty::Response::SharedPtr /*response*/)
        {
            RCLCPP_WARN(logger, "STOP MOTION requested — halting arm immediately");
            stop_requested.store(true);
            arm.stop();
        },
        rmw_qos_profile_services_default,
        stop_cb_group
    );

    RCLCPP_INFO(logger, "Services ready:");
    RCLCPP_INFO(logger, "  /joint_command         - Joint-space arm control");
    RCLCPP_INFO(logger, "  /cartesian_command     - Cartesian arm control");
    RCLCPP_INFO(logger, "  /set_octomap_enabled   - Enable/disable octomap for planning");
    RCLCPP_INFO(logger, "  /stop_motion           - Emergency stop current motion");
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
