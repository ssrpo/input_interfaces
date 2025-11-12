#include "joystick_interface/franka_gripper.hpp"

namespace input_interfaces
{

  FrankaGripper::FrankaGripper() : Node("franka_gripper")
  {
    // Read gripper parameters from config file
    // Using unique parameter names to avoid conflicts
    if (!this->has_parameter("gripper_closed_width"))
    {
      this->declare_parameter("gripper_closed_width", 0.00);
    }
    if (!this->has_parameter("gripper_open_width"))
    {
      this->declare_parameter("gripper_open_width", 0.05);
    }
    if (!this->has_parameter("gripper_closing_speed"))
    {
      this->declare_parameter("gripper_closing_speed", 0.03);
    }
    if (!this->has_parameter("gripper_closing_force"))
    {
      this->declare_parameter("gripper_closing_force", 50.0);
    }
    open_width_ = this->get_parameter("gripper_open_width").as_double();
    closed_width_ = this->get_parameter("gripper_closed_width").as_double();
    closing_speed_ = this->get_parameter("gripper_closing_speed").as_double();
    closing_force_ = this->get_parameter("gripper_closing_force").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Gripper parameters: closed_width: %.2f, closing_speed: %.2f, closing_force: %.2f",
                closed_width_, closing_speed_, closing_force_);

    teleop_cmd_subscriber_ = this->create_subscription<joystick_interface::msg::TeleopCmd>(
        "/teleop_cmd", 10,
        std::bind(&FrankaGripper::teleopCmdCallback, this, std::placeholders::_1));

    gripper_action_client_ =
        rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/franka_gripper/grasp");

    RCLCPP_INFO(this->get_logger(), "Franka gripper connected. Waiting for teleop commands...");
  }

  void FrankaGripper::teleopCmdCallback(const joystick_interface::msg::TeleopCmd::SharedPtr msg)
  {
    if (msg->gripper_cmd == joystick_interface::msg::TeleopCmd::GRIPPER_NO_CMD)
    {
      return;
    }

    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Gripper action server not available after waiting");
      return;
    }

    auto goal_msg =franka_msgs::action::Grasp::Goal();
    goal_msg.speed = closing_speed_;
    if (msg->gripper_cmd == joystick_interface::msg::TeleopCmd::GRIPPER_CMD_OPEN)
    {
      RCLCPP_INFO(this->get_logger(), "Franka gripper OPEN.");
      goal_msg.width = open_width_;
    }
    else if (msg->gripper_cmd == joystick_interface::msg::TeleopCmd::GRIPPER_CMD_CLOSE)
    {
      RCLCPP_INFO(this->get_logger(), "Franka gripper CLOSE.");
      goal_msg.width = closed_width_;
    }

    gripper_action_client_->async_send_goal(goal_msg);
  }
} // namespace input_interfaces

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input_interfaces::FrankaGripper>());
  rclcpp::shutdown();
  return 0;
}