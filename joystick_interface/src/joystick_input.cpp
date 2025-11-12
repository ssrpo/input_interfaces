#include "joystick_interface/joystick_input.hpp"

namespace input_interfaces
{
  JoystickInput::JoystickInput() : Node("joystick_input")
  {

    // Declare parameters for scaling factors with default values.
    this->declare_parameter<bool>("allow_full_mode",
                                  false); // to allow 6D teleoperation
    this->declare_parameter<std::string>("start_mode", "TRANSLATION_ROTATION");
    this->declare_parameter("linear_scale", 0.050);
    this->declare_parameter("z_axis_scale", 0.025);
    this->declare_parameter("angular_scale", 0.2);

    allow_full_mode_ = this->get_parameter("allow_full_mode").as_bool();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    z_axis_scale_ = this->get_parameter("z_axis_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();

    // Start mode logic
    std::string start_mode = this->get_parameter("start_mode").as_string();
    if (start_mode == "ROTATION")
      current_mode_ = Mode::ROTATION;
    else if (start_mode == "BOTH" && allow_full_mode_)
      current_mode_ = Mode::BOTH;
    else
      current_mode_ = Mode::TRANSLATION_ROTATION;

    RCLCPP_INFO(this->get_logger(), "Teleop starts in mode: %s",
                (current_mode_ == Mode::TRANSLATION_ROTATION ? "TRANSLATION_ROTATION"
                 : current_mode_ == Mode::ROTATION           ? "ROTATION"
                                                             : "BOTH (admin only)"));

    // Print the parameters on console.
    RCLCPP_INFO(this->get_logger(), "Joystick Controller - linear_scale: %.4f", linear_scale_);
    RCLCPP_INFO(this->get_logger(), "Joystick Controller - z_axis_scale: %.4f", z_axis_scale_);
    RCLCPP_INFO(this->get_logger(), "Joystick Controller - angular_scale: %.4f", angular_scale_);

    // Create a subscription to the SpaceMouse Joy messages on "/spacenav/joy".
    spacenav_joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/spacenav/joy", 10,
        std::bind(&JoystickInput::joyCallback, this, std::placeholders::_1));

    // Create a subscription to the 3D joystick Joy messages on "/joy".
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickInput::joy3dCallback, this, std::placeholders::_1));

    // Publisher for custom msg : Twist + teleop mode + gripper command
    teleop_cmd_publisher_ =
        this->create_publisher<joystick_interface::msg::TeleopCmd>("teleop_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Joystick Controller node initialized");
  }

  void JoystickInput::joy3dCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joystick_interface::msg::TeleopCmd cmd_msg;
    cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_NO_CMD;

    // Update current button states from message
    cur_button_gripper_ = msg->buttons[10];
    cur_button_1_ = msg->buttons[11];

    // --- Gripper command ---
    if (cur_button_gripper_ == 1 && last_button_gripper_ == 0)
    {
      if (!is_gripper_closed)
      {
        cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_CMD_CLOSE;
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_CLOSE");
        is_gripper_closed = true;
      }
      else
      {
        cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_CMD_OPEN;
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_OPEN");
        is_gripper_closed = false;
      }
    }

    // --- Mode switch: rising edge (single event per press) ---
    if (cur_button_1_ == 1 && last_button_1_ == 0)
    {
      if (allow_full_mode_)
      {
        // Cycle: TRANSLATION_ROTATION → ROTATION → TRANSLATION → BOTH →
        // TRANSLATION_ROTATION ...
        switch (current_mode_)
        {
        case Mode::TRANSLATION_ROTATION:
          current_mode_ = Mode::ROTATION;
          break;
        case Mode::ROTATION:
          current_mode_ = Mode::TRANSLATION;
          break;
        case Mode::TRANSLATION:
          current_mode_ = Mode::TRANSLATION_ROTATION;
          break;
        }
      }
      else
      {
        // Only cycle between TRANSLATION_ROTATION and ROTATION
        current_mode_ = (current_mode_ == Mode::TRANSLATION_ROTATION) ? Mode::ROTATION
                                                                      : Mode::TRANSLATION_ROTATION;
      }
      // Print info
      const char *mode_str = "";
      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
        mode_str = "TRANSLATION_ROTATION";
        break;
      case Mode::ROTATION:
        mode_str = "ROTATION";
        break;
      }
      RCLCPP_INFO(this->get_logger(), "Button 1: switched to mode %s", mode_str);
    }

    // --- Update button states (after all edge checks) ---
    last_button_gripper_ = cur_button_gripper_;
    last_button_1_ = cur_button_1_;

    // --- Compute and publish Twist ---
    auto twist = geometry_msgs::msg::Twist();
    if (msg->axes.size() >= 3)
    {
      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
        twist.linear.x = linear_scale_ * msg->axes[1];
        twist.linear.y = linear_scale_ * msg->axes[0];
        twist.linear.z = z_axis_scale_ * msg->axes[2];
        break;
      case Mode::ROTATION:
        twist.angular.x = -angular_scale_ * msg->axes[0];
        twist.angular.y = -angular_scale_ * msg->axes[1];
        twist.angular.z = -angular_scale_ * msg->axes[2];
        break;

      case Mode::TRANSLATION:
        twist.linear.x = linear_scale_ * msg->axes[1];
        twist.linear.y = linear_scale_ * msg->axes[0];
        twist.linear.z = z_axis_scale_ * msg->axes[2];
        break;
      }
    }

    // --- Publish custom TeleopCmd msg ---
    cmd_msg.twist = twist;
    cmd_msg.mode = static_cast<uint8_t>(current_mode_);
    teleop_cmd_publisher_->publish(cmd_msg);
  }

  void JoystickInput::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joystick_interface::msg::TeleopCmd cmd_msg;
    cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_NO_CMD;
    
    // Update current button states from message
    cur_button_gripper_ = msg->buttons[0];
    cur_button_1_ = msg->buttons[1];

    // --- Gripper command ---
    if (cur_button_gripper_ == 1 && last_button_gripper_ == 0)
    {
      if (!is_gripper_closed)
      {
        cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_CMD_CLOSE;
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_CLOSE");
        is_gripper_closed = true;
      }
      else
      {
        cmd_msg.gripper_cmd = joystick_interface::msg::TeleopCmd::GRIPPER_CMD_OPEN;
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_OPEN");
        is_gripper_closed = false;
      }
    }

    // --- Mode switch: rising edge (single event per press) ---
    if (cur_button_1_ == 1 && last_button_1_ == 0)
    {
      if (allow_full_mode_)
      {
        // Cycle: TRANSLATION_ROTATION → ROTATION → TRANSLATION → BOTH →
        // TRANSLATION_ROTATION ...
        switch (current_mode_)
        {
        case Mode::TRANSLATION_ROTATION:
          current_mode_ = Mode::ROTATION;
          break;
        case Mode::ROTATION:
          current_mode_ = Mode::TRANSLATION;
          break;
        case Mode::TRANSLATION:
          current_mode_ = Mode::TRANSLATION_ROTATION;
          break;
        }
      }
      else
      {
        // Only cycle between TRANSLATION_ROTATION and ROTATION
        current_mode_ = (current_mode_ == Mode::TRANSLATION_ROTATION) ? Mode::ROTATION
                                                                      : Mode::TRANSLATION_ROTATION;
      }
      // Print info
      const char *mode_str = "";
      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
        mode_str = "TRANSLATION_ROTATION";
        break;
      case Mode::ROTATION:
        mode_str = "ROTATION";
        break;
      }
      RCLCPP_INFO(this->get_logger(), "Button 1: switched to mode %s", mode_str);
    }

    // --- Update button states (after all edge checks) ---
    last_button_gripper_ = cur_button_gripper_;
    last_button_1_ = cur_button_1_;

    // --- Compute and publish Twist ---
    auto twist = geometry_msgs::msg::Twist();
    if (msg->axes.size() >= 6)
    {
      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
        twist.linear.x = linear_scale_ * msg->axes[4];
        twist.linear.y = -linear_scale_ * msg->axes[3];
        twist.linear.z = z_axis_scale_ * msg->axes[5];
        break;
      case Mode::ROTATION:
        twist.angular.x = angular_scale_ * msg->axes[3];
        twist.angular.y = -angular_scale_ * msg->axes[4];
        twist.angular.z = -angular_scale_ * msg->axes[5];
        break;
      case Mode::TRANSLATION:
        twist.linear.x = linear_scale_ * msg->axes[4];
        twist.linear.y = -linear_scale_ * msg->axes[3];
        twist.linear.z = z_axis_scale_ * msg->axes[5];
        break;
      case Mode::BOTH:
        twist.linear.x = linear_scale_ * msg->axes[0];
        twist.linear.y = linear_scale_ * msg->axes[1];
        twist.linear.z = z_axis_scale_ * msg->axes[2];
        twist.angular.x = angular_scale_ * msg->axes[3];
        twist.angular.y = angular_scale_ * msg->axes[4];
        twist.angular.z = angular_scale_ * msg->axes[5];
        break;
      }
    }

    // --- Publish custom TeleopCmd msg ---
    cmd_msg.twist = twist;
    cmd_msg.mode = static_cast<uint8_t>(current_mode_);
    teleop_cmd_publisher_->publish(cmd_msg);
  }
} // namespace input_interfaces
