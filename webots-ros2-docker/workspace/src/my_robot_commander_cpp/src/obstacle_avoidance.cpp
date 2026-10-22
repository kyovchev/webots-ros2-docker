#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("collision_avoidance");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // --- Wait for obstacle pose from topic ---
    geometry_msgs::msg::PoseStamped::SharedPtr detected_object = nullptr;

    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/detected_object", 1,
        [&detected_object](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            detected_object = msg;
        });

    // Block until we receive one message
    RCLCPP_INFO(node->get_logger(), "Waiting for obstacle pose on /detected_object ...");
    while (rclcpp::ok() && !detected_object) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "Obstacle received: x=%.2f y=%.2f z=%.2f",
        detected_object->pose.position.x,
        detected_object->pose.position.y,
        detected_object->pose.position.z);

    // --- Add obstacle using received pose ---
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = arm.getPlanningFrame();
    box.id = "obstacle_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.05;

    // Use the received pose directly
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x    = detected_object->pose.position.x;
    box_pose.position.y    = detected_object->pose.position.y;
    box_pose.position.z    = detected_object->pose.position.z;
    box_pose.orientation.w = 1.0;  // no rotation

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    planning_scene.applyCollisionObject(box);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // --- Plan to a pose goal (arm will avoid the box) ---
    std::vector<double> joints = { 0, -1, 0, 0, 0, 0 };
    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        arm.execute(plan1);
    }

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("Home");
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success2) {
        arm.execute(plan2);
    }

    // --- Clean up ---
    // planning_scene.removeCollisionObjects({"obstacle_box"});

    rclcpp::shutdown();
    spinner.join();
    return 0;
}