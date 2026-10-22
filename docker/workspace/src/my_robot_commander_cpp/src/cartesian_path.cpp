#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // Pose Goal

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = -0.05;
    target_pose.pose.position.y = 0.12;
    target_pose.pose.position.z = 0.05;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        arm.execute(plan);
    }

    // Cartesian Path

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.x += 0.1;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.z += 0.05;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.x -= 0.1;
    pose3.position.z -= 0.05;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction == 1) {
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}