#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <my_robot_rviz_panel/panel.hpp>

namespace my_robot_rviz_panel
{
MyRobotPanel::MyRobotPanel(QWidget * parent) : Panel(parent)
{
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("Pick and Place");
  layout->addWidget(label_);
  layout->addWidget(button_);

  QObject::connect(button_, &QPushButton::released, this, &MyRobotPanel::sendPickAndPlaceGoal);
}

MyRobotPanel::~MyRobotPanel() = default;

void MyRobotPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/status_pick_and_place", 1, std::bind(&MyRobotPanel::topicCallback, this, std::placeholders::_1));
  action_client_ = rclcpp_action::create_client<my_robot_interfaces::action::PickAndPlace>(node, "/pick_and_place");
}

void MyRobotPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString::fromStdString(msg.data.c_str()));
}

void MyRobotPanel::sendPickAndPlaceGoal()
{
  if (!action_client_->wait_for_action_server(
        std::chrono::seconds(1)))
  {
    label_->setText("Action server not available");
    return;
  }

  my_robot_interfaces::action::PickAndPlace::Goal goal_msg;

  auto send_goal_options =
    rclcpp_action::Client<my_robot_interfaces::action::PickAndPlace>::SendGoalOptions();

  action_client_->async_send_goal(
    goal_msg,
    send_goal_options);

  label_->setText("Pick and place goal sent");
}

}  // namespace my_robot_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_robot_rviz_panel::MyRobotPanel, rviz_common::Panel)
