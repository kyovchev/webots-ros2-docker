#ifndef MY_ROBOT_RVIZ_PANEL__MY_ROBOT_PANEL_HPP_
#define MY_ROBOT_RVIZ_PANEL__MY_ROBOT_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <my_robot_interfaces/action/pick_and_place.hpp>


namespace my_robot_rviz_panel
{
class MyRobotPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit MyRobotPanel(QWidget * parent = 0);
  ~MyRobotPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp_action::Client<my_robot_interfaces::action::PickAndPlace>::SharedPtr action_client_;
  void topicCallback(const std_msgs::msg::String& msg);

  QLabel * label_;
  QPushButton * button_;

private Q_SLOTS:
  void sendPickAndPlaceGoal();
};

}  // namespace my_robot_rviz_panel

#endif  // MY_ROBOT_RVIZ_PANEL__MY_ROBOT_PANEL_HPP_
