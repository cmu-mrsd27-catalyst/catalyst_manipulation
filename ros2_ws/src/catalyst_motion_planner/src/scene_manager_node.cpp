#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <catalyst_interfaces/srv/json_command.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <map>

using json = nlohmann::json;
using JsonCommand = catalyst_interfaces::srv::JsonCommand;

static const std::string BASE_FRAME = "link_base";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("scene_manager", node_options);
    auto logger = node->get_logger();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    // Initialize PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface psi;
    RCLCPP_INFO(logger, "PlanningSceneInterface initialized");

    // Track known object IDs
    std::map<std::string, moveit_msgs::msg::CollisionObject> known_objects;
    std::mutex objects_mutex;

    // ── Auto-load world objects from YAML ──
    double base_frame_z = node->get_parameter_or(
        "base_frame_z", rclcpp::Parameter("base_frame_z", 0.80)).as_double();

    std::string world_yaml_path = node->get_parameter_or(
        "world_objects_yaml", rclcpp::Parameter("world_objects_yaml", "")).as_string();

    if (!world_yaml_path.empty()) {
        try {
            RCLCPP_INFO(logger, "Loading world objects from: %s", world_yaml_path.c_str());
            YAML::Node config = YAML::LoadFile(world_yaml_path);

            std::vector<moveit_msgs::msg::CollisionObject> objects;

            // Robot stand
            if (config["robot_stand"]) {
                auto stand = config["robot_stand"];
                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = BASE_FRAME;
                obj.id = "robot_stand";
                obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                shape_msgs::msg::SolidPrimitive prim;
                prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                prim.dimensions = {
                    stand["size"]["x"].as<double>(),
                    stand["size"]["y"].as<double>(),
                    stand["size"]["z"].as<double>()
                };

                geometry_msgs::msg::Pose pose;
                pose.position.x = stand["position"]["x"].as<double>();
                pose.position.y = stand["position"]["y"].as<double>();
                pose.position.z = stand["position"]["z"].as<double>() - base_frame_z;
                pose.orientation.w = 1.0;

                obj.primitives.push_back(prim);
                obj.primitive_poses.push_back(pose);
                objects.push_back(obj);
                known_objects[obj.id] = obj;
                RCLCPP_INFO(logger, "  Added: robot_stand (box %.2fx%.2fx%.2f at z=%.2f)",
                            prim.dimensions[0], prim.dimensions[1], prim.dimensions[2],
                            pose.position.z);
            }

            // Table top
            if (config["work_table"]) {
                auto table = config["work_table"];
                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = BASE_FRAME;
                obj.id = "table_top";
                obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                shape_msgs::msg::SolidPrimitive prim;
                prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                prim.dimensions = {
                    table["top"]["x"].as<double>(),
                    table["top"]["y"].as<double>(),
                    table["top"]["z"].as<double>()
                };

                // Table top center in world frame: (table.x, table.y, stand_height - thickness/2)
                double table_top_z_world = base_frame_z - table["top"]["z"].as<double>() / 2.0;
                geometry_msgs::msg::Pose pose;
                pose.position.x = table["position"]["x"].as<double>();
                pose.position.y = table["position"]["y"].as<double>();
                pose.position.z = table_top_z_world - base_frame_z;
                pose.orientation.w = 1.0;

                obj.primitives.push_back(prim);
                obj.primitive_poses.push_back(pose);
                objects.push_back(obj);
                known_objects[obj.id] = obj;
                RCLCPP_INFO(logger, "  Added: table_top (box %.2fx%.2fx%.2f at z=%.2f)",
                            prim.dimensions[0], prim.dimensions[1], prim.dimensions[2],
                            pose.position.z);
            }

            // Boxes defined in link_base (center pose, sizes along base X/Y/Z)
            if (config["link_base_boxes"]) {
                for (const auto& entry : config["link_base_boxes"]) {
                    moveit_msgs::msg::CollisionObject obj;
                    obj.header.frame_id = BASE_FRAME;
                    obj.id = entry["id"].as<std::string>();
                    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                    shape_msgs::msg::SolidPrimitive prim;
                    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                    prim.dimensions = {
                        entry["size"]["x"].as<double>(),
                        entry["size"]["y"].as<double>(),
                        entry["size"]["z"].as<double>(),
                    };

                    geometry_msgs::msg::Pose pose;
                    pose.position.x = entry["position"]["x"].as<double>();
                    pose.position.y = entry["position"]["y"].as<double>();
                    pose.position.z = entry["position"]["z"].as<double>();
                    pose.orientation.w = 1.0;
                    if (entry["orientation"]) {
                        const auto o = entry["orientation"];
                        pose.orientation.x = o["qx"].as<double>();
                        pose.orientation.y = o["qy"].as<double>();
                        pose.orientation.z = o["qz"].as<double>();
                        pose.orientation.w = o["qw"].as<double>();
                    }

                    obj.primitives.push_back(prim);
                    obj.primitive_poses.push_back(pose);
                    objects.push_back(obj);
                    known_objects[obj.id] = obj;
                    RCLCPP_INFO(
                        logger,
                        "  Added link_base box: %s (%.3fx%.3fx%.3f m, center %.3f, %.3f, %.3f)",
                        obj.id.c_str(), prim.dimensions[0], prim.dimensions[1],
                        prim.dimensions[2], pose.position.x, pose.position.y,
                        pose.position.z);
                }
            }

            // Ground plane
            {
                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = BASE_FRAME;
                obj.id = "ground_plane";
                obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                shape_msgs::msg::SolidPrimitive prim;
                prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                prim.dimensions = {10.0, 10.0, 0.01};

                geometry_msgs::msg::Pose pose;
                pose.position.x = 0.0;
                pose.position.y = 0.0;
                pose.position.z = -base_frame_z - 0.005;
                pose.orientation.w = 1.0;

                obj.primitives.push_back(prim);
                obj.primitive_poses.push_back(pose);
                objects.push_back(obj);
                known_objects[obj.id] = obj;
                RCLCPP_INFO(logger, "  Added: ground_plane at z=%.2f", pose.position.z);
            }

            // Apply all objects synchronously
            psi.applyCollisionObjects(objects);
            RCLCPP_INFO(logger, "World objects loaded (%zu objects)", objects.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Failed to load world objects YAML: %s", e.what());
        }
    } else {
        RCLCPP_WARN(logger, "No world_objects_yaml parameter set, skipping auto-load");
    }

    auto get_planning_scene_client =
        node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    // ── Scene Command Service ──
    auto scene_service = node->create_service<JsonCommand>(
        "/scene_command",
        [&psi, &known_objects, &objects_mutex, &logger, get_planning_scene_client](
            const JsonCommand::Request::SharedPtr request,
            JsonCommand::Response::SharedPtr response)
        {
            try {
                auto cmd = json::parse(request->command);
                std::string action = cmd.at("action").get<std::string>();

                if (action == "add") {
                    std::string id = cmd.at("id").get<std::string>();
                    std::string shape = cmd.at("shape").get<std::string>();
                    std::string frame_id = cmd.value("frame_id", BASE_FRAME);

                    auto pos = cmd.at("position").get<std::vector<double>>();
                    if (pos.size() != 3) {
                        response->response = json({{"success", false},
                            {"message", "position must have 3 elements"}}).dump();
                        return;
                    }

                    std::vector<double> orient = cmd.value("orientation", std::vector<double>{0, 0, 0, 1});
                    if (orient.size() != 4) {
                        response->response = json({{"success", false},
                            {"message", "orientation must have 4 elements [qx,qy,qz,qw]"}}).dump();
                        return;
                    }

                    moveit_msgs::msg::CollisionObject obj;
                    obj.header.frame_id = frame_id;
                    obj.id = id;
                    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                    geometry_msgs::msg::Pose pose;
                    pose.position.x = pos[0];
                    pose.position.y = pos[1];
                    pose.position.z = pos[2];
                    pose.orientation.x = orient[0];
                    pose.orientation.y = orient[1];
                    pose.orientation.z = orient[2];
                    pose.orientation.w = orient[3];

                    if (shape == "mesh") {
                        std::string mesh_path = cmd.at("mesh_path").get<std::string>();
                        std::vector<double> scale = cmd.value("scale", std::vector<double>{1.0, 1.0, 1.0});
                        if (scale.size() != 3) {
                            response->response = json({{"success", false},
                                {"message", "scale must have 3 elements"}}).dump();
                            return;
                        }

                        Eigen::Vector3d scale_vec(scale[0], scale[1], scale[2]);
                        shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale_vec);
                        if (!mesh) {
                            response->response = json({{"success", false},
                                {"message", "Failed to load mesh from: " + mesh_path}}).dump();
                            return;
                        }

                        shape_msgs::msg::Mesh mesh_msg;
                        shapes::ShapeMsg shape_msg;
                        shapes::constructMsgFromShape(mesh, shape_msg);
                        mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
                        delete mesh;

                        obj.meshes.push_back(mesh_msg);
                        obj.mesh_poses.push_back(pose);
                    } else {
                        auto dims = cmd.at("dimensions").get<std::vector<double>>();

                        shape_msgs::msg::SolidPrimitive prim;
                        if (shape == "box") {
                            if (dims.size() != 3) {
                                response->response = json({{"success", false},
                                    {"message", "box requires 3 dimensions [x,y,z]"}}).dump();
                                return;
                            }
                            prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                            prim.dimensions.assign(dims.begin(), dims.end());
                        } else if (shape == "cylinder") {
                            if (dims.size() != 2) {
                                response->response = json({{"success", false},
                                    {"message", "cylinder requires 2 dimensions [height,radius]"}}).dump();
                                return;
                            }
                            prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                            prim.dimensions.assign(dims.begin(), dims.end());
                        } else if (shape == "sphere") {
                            if (dims.size() != 1) {
                                response->response = json({{"success", false},
                                    {"message", "sphere requires 1 dimension [radius]"}}).dump();
                                return;
                            }
                            prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                            prim.dimensions.assign(dims.begin(), dims.end());
                        } else {
                            response->response = json({{"success", false},
                                {"message", "Unknown shape: " + shape + ". Use box/cylinder/sphere/mesh"}}).dump();
                            return;
                        }

                        obj.primitives.push_back(prim);
                        obj.primitive_poses.push_back(pose);
                    }

                    psi.applyCollisionObject(obj);
                    {
                        std::lock_guard<std::mutex> lock(objects_mutex);
                        known_objects[id] = obj;
                    }
                    RCLCPP_INFO(logger, "Added collision object: %s (%s)", id.c_str(), shape.c_str());
                    response->response = json({{"success", true},
                        {"message", "Added object: " + id}}).dump();

                } else if (action == "remove") {
                    std::string id = cmd.at("id").get<std::string>();
                    moveit_msgs::msg::CollisionObject obj;
                    obj.header.frame_id = BASE_FRAME;
                    obj.id = id;
                    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                    psi.applyCollisionObject(obj);
                    {
                        std::lock_guard<std::mutex> lock(objects_mutex);
                        known_objects.erase(id);
                    }
                    RCLCPP_INFO(logger, "Removed collision object: %s", id.c_str());
                    response->response = json({{"success", true},
                        {"message", "Removed object: " + id}}).dump();

                } else if (action == "move") {
                    std::string id = cmd.at("id").get<std::string>();
                    auto pos = cmd.at("position").get<std::vector<double>>();
                    if (pos.size() != 3) {
                        response->response = json({{"success", false},
                            {"message", "position must have 3 elements"}}).dump();
                        return;
                    }
                    std::vector<double> orient = cmd.value("orientation", std::vector<double>{0, 0, 0, 1});

                    std::lock_guard<std::mutex> lock(objects_mutex);
                    auto it = known_objects.find(id);
                    if (it == known_objects.end()) {
                        response->response = json({{"success", false},
                            {"message", "Unknown object: " + id}}).dump();
                        return;
                    }

                    // Update the stored object and re-apply
                    auto& obj = it->second;
                    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

                    geometry_msgs::msg::Pose new_pose;
                    new_pose.position.x = pos[0];
                    new_pose.position.y = pos[1];
                    new_pose.position.z = pos[2];
                    new_pose.orientation.x = orient[0];
                    new_pose.orientation.y = orient[1];
                    new_pose.orientation.z = orient[2];
                    new_pose.orientation.w = orient[3];

                    if (!obj.primitive_poses.empty()) {
                        obj.primitive_poses[0] = new_pose;
                    } else if (!obj.mesh_poses.empty()) {
                        obj.mesh_poses[0] = new_pose;
                    }

                    psi.applyCollisionObject(obj);
                    RCLCPP_INFO(logger, "Moved collision object: %s to (%.3f, %.3f, %.3f)",
                                id.c_str(), pos[0], pos[1], pos[2]);
                    response->response = json({{"success", true},
                        {"message", "Moved object: " + id}}).dump();

                } else if (action == "attach") {
                    std::string id = cmd.at("id").get<std::string>();
                    std::string link = cmd.at("link").get<std::string>();

                    moveit_msgs::msg::AttachedCollisionObject aco;
                    aco.link_name = link;
                    aco.object.id = id;
                    aco.object.header.frame_id = BASE_FRAME;
                    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

                    psi.applyAttachedCollisionObject(aco);
                    RCLCPP_INFO(logger, "Attached object '%s' to link '%s'", id.c_str(), link.c_str());
                    response->response = json({{"success", true},
                        {"message", "Attached " + id + " to " + link}}).dump();

                } else if (action == "detach") {
                    std::string id = cmd.at("id").get<std::string>();

                    moveit_msgs::msg::AttachedCollisionObject aco;
                    aco.object.id = id;
                    aco.object.header.frame_id = BASE_FRAME;
                    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

                    psi.applyAttachedCollisionObject(aco);
                    RCLCPP_INFO(logger, "Detached object: %s", id.c_str());
                    response->response = json({{"success", true},
                        {"message", "Detached object: " + id}}).dump();

                } else if (action == "clear") {
                    // Remove all known objects
                    std::vector<std::string> ids;
                    {
                        std::lock_guard<std::mutex> lock(objects_mutex);
                        for (const auto& [id, _] : known_objects) {
                            ids.push_back(id);
                        }
                        known_objects.clear();
                    }
                    if (!ids.empty()) {
                        std::vector<moveit_msgs::msg::CollisionObject> remove_objs;
                        for (const auto& id : ids) {
                            moveit_msgs::msg::CollisionObject obj;
                            obj.header.frame_id = BASE_FRAME;
                            obj.id = id;
                            obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                            remove_objs.push_back(obj);
                        }
                        psi.applyCollisionObjects(remove_objs);
                    }
                    RCLCPP_INFO(logger, "Cleared all collision objects (%zu removed)", ids.size());
                    response->response = json({{"success", true},
                        {"message", "Cleared " + std::to_string(ids.size()) + " objects"}}).dump();

                } else if (action == "list") {
                    json id_list = json::array();
                    {
                        std::lock_guard<std::mutex> lock(objects_mutex);
                        for (const auto& [id, _] : known_objects) {
                            id_list.push_back(id);
                        }
                    }
                    RCLCPP_INFO(logger, "List: %zu objects", id_list.size());
                    response->response = json({{"success", true},
                        {"objects", id_list}}).dump();

                } else if (action == "allow_object_default_collisions") {
                    std::string object_id = cmd.at("object_id").get<std::string>();
                    bool allow = cmd.at("allow").get<bool>();

                    if (!get_planning_scene_client->wait_for_service(std::chrono::seconds(5))) {
                        response->response = json({{"success", false},
                            {"message", "get_planning_scene service not available"}}).dump();
                        return;
                    }

                    auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
                    get_req->components.components = 128;  // ALLOWED_COLLISION_MATRIX
                    auto future = get_planning_scene_client->async_send_request(get_req);
                    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
                        response->response = json({{"success", false},
                            {"message", "GetPlanningScene timed out"}}).dump();
                        return;
                    }

                    moveit_msgs::msg::AllowedCollisionMatrix acm =
                        future.get()->scene.allowed_collision_matrix;
                    if (acm.entry_names.empty()) {
                        response->response = json({{"success", false},
                            {"message", "Planning scene ACM is empty"}}).dump();
                        return;
                    }

                    bool found_default = false;
                    for (size_t i = 0; i < acm.default_entry_names.size(); ++i) {
                        if (acm.default_entry_names[i] == object_id) {
                            acm.default_entry_values[i] = allow;
                            found_default = true;
                            break;
                        }
                    }
                    if (!found_default) {
                        acm.default_entry_names.push_back(object_id);
                        acm.default_entry_values.push_back(allow);
                    }

                    moveit_msgs::msg::PlanningScene ps;
                    ps.is_diff = true;
                    ps.allowed_collision_matrix = acm;

                    bool applied = psi.applyPlanningScene(ps);
                    if (applied) {
                        RCLCPP_INFO(logger, "ACM default for '%s': allow_all=%s",
                                    object_id.c_str(), allow ? "true" : "false");
                        response->response = json({{"success", true},
                            {"message", "Updated ACM for " + object_id}}).dump();
                    } else {
                        response->response = json({{"success", false},
                            {"message", "applyPlanningScene failed"}}).dump();
                    }

                } else {
                    response->response = json({{"success", false},
                        {"message", "Unknown action: " + action +
                                    ". Use add/remove/move/attach/detach/clear/list/"
                                    "allow_object_default_collisions"}}).dump();
                }

            } catch (const json::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("JSON error: ") + e.what()}}).dump();
            } catch (const std::exception& e) {
                response->response = json({{"success", false},
                    {"message", std::string("Error: ") + e.what()}}).dump();
            }
        }
    );

    RCLCPP_INFO(logger, "Scene manager ready:");
    RCLCPP_INFO(logger,
                "  /scene_command - add/remove/move/attach/detach/clear/list/"
                "allow_object_default_collisions");

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
