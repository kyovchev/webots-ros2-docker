#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace webots_ros2_hw
{

class TopicHardware : public hardware_interface::SystemInterface
{
public:
  TopicHardware();

  // ── Lifecycle ────────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interfaces ───────────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Read / Write ─────────────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t NUM_JOINTS = 6;

  // Configurable parameters (set via <param> in the URDF/xacro)
  std::string cmd_topic_;
  std::array<std::string, NUM_JOINTS> state_topics_;
  double cmd_timeout_sec_;

  // Joint state / command storage (indexed 0-5)
  std::array<double, NUM_JOINTS> hw_positions_{};
  std::array<double, NUM_JOINTS> hw_velocities_{};
  std::array<double, NUM_JOINTS> hw_commands_{};

  // Last time a state message was received per joint
  std::array<rclcpp::Time, NUM_JOINTS> last_state_time_;

  // ROS 2 node used by this hardware plugin
  rclcpp::Node::SharedPtr node_;

  // Publisher for the command topic
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;

  // One subscriber per joint
  std::array<
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr,
    NUM_JOINTS>
    state_subs_;

  // Executor running the node in a background thread
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // Helper
  void build_default_topics();
};

}  // namespace webots_ros2_hw