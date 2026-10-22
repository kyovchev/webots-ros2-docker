#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <my_robot_interfaces/msg/joints_command.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <std_msgs/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using JointsCommand = my_robot_interfaces::msg::JointsCommand;
using PoseCommand   = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_  = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        is_moving_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "is_moving", rclcpp::QoS(1).transient_local());
        publishMoving(false);

        joint_cmd_sub_ = node_->create_subscription<JointsCommand>(
            "joint_command", 1, std::bind(&Commander::jointCmdCallback, this, _1));
        pose_cmd_sub_  = node_->create_subscription<PoseCommand>(
            "pose_command",  1, std::bind(&Commander::poseCmdCallback,  this, _1));
    }

private:
    void publishMoving(bool moving)
    {
        std_msgs::msg::Bool msg;
        msg.data = moving;
        is_moving_pub_->publish(msg);
    }

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            publishMoving(true);
            interface->execute(plan);
            publishMoving(false);
        }
    }

    void jointCmdCallback(const JointsCommand &msg)
    {
        auto joints = msg.data;
        if (joints.size() == 6) {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);
            planAndExecute(arm_);
        }
    }

    void poseCmdCallback(const PoseCommand &msg)
    {
        tf2::Quaternion q;
        q.setRPY(msg.roll, msg.pitch, msg.yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id        = "base_link";
        target_pose.pose.position.x        = msg.x;
        target_pose.pose.position.y        = msg.y;
        target_pose.pose.position.z        = msg.z;
        target_pose.pose.orientation.x     = q.getX();
        target_pose.pose.orientation.y     = q.getY();
        target_pose.pose.orientation.z     = q.getZ();
        target_pose.pose.orientation.w     = q.getW();

        arm_->setStartStateToCurrentState();

        if (!msg.cartesian_path) {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        } else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction == 1) {
                publishMoving(true);
                arm_->execute(trajectory);
                publishMoving(false);
            }
        }
    }

    std::shared_ptr<rclcpp::Node>         node_;
    std::shared_ptr<MoveGroupInterface>   arm_;
    rclcpp::Subscription<JointsCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr   pose_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_moving_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node      = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}