#include "webots_ros2_hw/topic_hardware.hpp"

#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

// ---------------------------------------------------------------------------
// Register the plugin with pluginlib
// ---------------------------------------------------------------------------
PLUGINLIB_EXPORT_CLASS(
  webots_ros2_hw::TopicHardware,
  hardware_interface::SystemInterface)

namespace webots_ros2_hw
{

// ── Constructor ─────────────────────────────────────────────────────────────
TopicHardware::TopicHardware()
: cmd_topic_("/my_robot_cmd"),
  cmd_timeout_sec_(1.0)
{
  build_default_topics();
}

void TopicHardware::build_default_topics()
{
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    state_topics_[i] = "joint" + std::to_string(i);
  }
}

// ── on_init ─────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn TopicHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Validate URDF joint count ───────────────────────────────────────────
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("TopicHardware"),
      "Expected exactly %zu joints in the URDF, got %zu.",
      NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Validate each joint has position state + position command interface ──
  for (const auto & joint : info_.joints) {
    bool has_pos_state = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        has_pos_state = true;
      }
    }
    bool has_pos_cmd = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        has_pos_cmd = true;
      }
    }
    if (!has_pos_state || !has_pos_cmd) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TopicHardware"),
        "Joint '%s' must have both a position state interface and a "
        "position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ── Read optional <param> overrides from the URDF ──────────────────────
  // Example:
  //   <param name="cmd_topic">/my_robot_cmd</param>
  //   <param name="state_topic_0">joint0</param>
  //   <param name="cmd_timeout">1.0</param>

  auto get_param = [&](const std::string & key,
    const std::string & default_val) -> std::string {
      auto it = info_.hardware_parameters.find(key);
      return (it != info_.hardware_parameters.end()) ? it->second : default_val;
    };

  cmd_topic_ = get_param("cmd_topic", cmd_topic_);
  cmd_timeout_sec_ = std::stod(get_param("cmd_timeout", std::to_string(cmd_timeout_sec_)));

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    state_topics_[i] = get_param(
      "state_topic_" + std::to_string(i),
      state_topics_[i]);
  }

  // Initialise storage to zero
  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_commands_.fill(0.0);

  RCLCPP_INFO(rclcpp::get_logger("TopicHardware"), "on_init() OK");
  RCLCPP_INFO(rclcpp::get_logger("TopicHardware"), "  cmd_topic   : %s", cmd_topic_.c_str());
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("TopicHardware"),
      "  state_topic[%zu]: %s", i, state_topics_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn TopicHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create a dedicated internal node so we can spin independently
  node_ = rclcpp::Node::make_shared("topic_hardware_node");

  // Command publisher
  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    cmd_topic_, rclcpp::QoS(1));

  // Per-joint state subscribers
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    const std::size_t idx = i;   // capture by value
    state_subs_[i] = node_->create_subscription<std_msgs::msg::Float32>(
      state_topics_[i],
      rclcpp::QoS(1),
      [this, idx](const std_msgs::msg::Float32::SharedPtr msg) {
        hw_positions_[idx] = static_cast<double>(msg->data);
        last_state_time_[idx] = node_->now();
      });

    // Initialise timestamp to "zero" so timeout logic works before first msg
    last_state_time_[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  // Spin the node in a background thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("TopicHardware"), "on_configure() OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn TopicHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Seed commands with the current measured positions so the robot does not
  // jump when the controller first takes over.
  hw_commands_ = hw_positions_;

  RCLCPP_INFO(rclcpp::get_logger("TopicHardware"), "on_activate() OK – hardware is ACTIVE");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn TopicHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the background executor
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  RCLCPP_INFO(rclcpp::get_logger("TopicHardware"), "on_deactivate() OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ──────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
TopicHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(NUM_JOINTS * 2);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    // Position
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    // Velocity (kept at 0 – extend if your robot reports velocity)
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }
  return interfaces;
}

// ── export_command_interfaces ────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
TopicHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return interfaces;
}

// ── read ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type TopicHardware::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  // Data arrives in the background executor thread.
  // hw_positions_ is already updated by the subscriber callbacks.
  // Warn if any joint has not reported recently.

  if (cmd_timeout_sec_ > 0.0) {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
      if (last_state_time_[i].nanoseconds() == 0) {
        // No message received yet – not an error at startup
        continue;
      }
      const double age = (time - last_state_time_[i]).seconds();
      if (age > cmd_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("TopicHardware"),
          *node_->get_clock(), 2000,
          "Joint %zu state topic '%s' stale (%.2f s)",
          i, state_topics_[i].c_str(), age);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type TopicHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Pack all joint commands into a single Float32MultiArray message.
  // Format expected by the robot driver:
  //   data[0]   = enable flag  (1 = enabled)
  //   data[1-6] = joint0..joint5 position commands (radians)
  //
  // This matches the example:
  //   ros2 topic pub -1 /my_robot_cmd std_msgs/msg/Float32MultiArray
  //   "data: [1, 1, 0, 0, 0, 0, 0]"
  //            ^  ^------- joint positions
  //            enable flag

  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve(NUM_JOINTS + 1);

  // Enable flag – always 1 while active
  msg.data.push_back(1.0f);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    msg.data.push_back(static_cast<float>(hw_commands_[i]));
  }

  cmd_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace webots_ros2_hw