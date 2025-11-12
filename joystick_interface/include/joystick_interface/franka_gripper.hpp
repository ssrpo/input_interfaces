#pragma once

#include "franka_msgs/action/grasp.hpp"
#include "joystick_interface/msg/teleop_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace input_interfaces
{
  class FrankaGripper : public rclcpp::Node
  {
  public:
    FrankaGripper();

    void teleopCmdCallback(const joystick_interface::msg::TeleopCmd::SharedPtr msg);

  private:
    rclcpp::Subscription<joystick_interface::msg::TeleopCmd>::SharedPtr teleop_cmd_subscriber_;
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_action_client_;

    // Gripper parameters.
    double open_width_;
    double closed_width_;
    double closing_speed_;
    double closing_force_;
  };
} // namespace input_interfaces