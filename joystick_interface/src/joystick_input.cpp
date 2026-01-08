#include "joystick_interface/joystick_input.hpp"

namespace input_interfaces
{
  JoystickInput::JoystickInput() : Node("joystick_input")
  {
    typedef extender_msgs::msg::TeleopCommand Mode;

    // Declare parameters for scaling factors with default values.
    this->declare_parameter<bool>("allow_full_mode",
                                  false); // to allow 6D teleoperation
    this->declare_parameter<std::string>("start_mode", "TRANSLATION_ROTATION");
    this->declare_parameter("linear_scale", 0.050);
    this->declare_parameter("z_axis_scale", 0.025);
    this->declare_parameter("angular_scale", 0.2);

    // Vector parameters for joystick (/joy) mapping
    this->declare_parameter<std::vector<int>>("joy_translation_axes",
                                              mapping_.joy_translation_axes);
    this->declare_parameter<std::vector<int>>("joy_rotation_axes", mapping_.joy_rotation_axes);
    this->declare_parameter<std::vector<double>>("joy_translation_signs",
                                                 mapping_.joy_translation_signs);
    this->declare_parameter<std::vector<double>>("joy_rotation_signs", mapping_.joy_rotation_signs);

    // Vector parameters for SpaceMouse (/spacenav/joy) mapping
    this->declare_parameter<std::vector<int>>("spacenav_translation_axes",
                                              mapping_.spacenav_translation_axes);
    this->declare_parameter<std::vector<int>>("spacenav_rotation_axes",
                                              mapping_.spacenav_rotation_axes);
    this->declare_parameter<std::vector<double>>("spacenav_translation_signs",
                                                 mapping_.spacenav_translation_signs);
    this->declare_parameter<std::vector<double>>("spacenav_rotation_signs",
                                                 mapping_.spacenav_rotation_signs);

    this->declare_parameter("mode_button_joy", mapping_.mode_button_joy);
    this->declare_parameter("mode_button_spacenav", mapping_.mode_button_spacenav);

    allow_full_mode_ = this->get_parameter("allow_full_mode").as_bool();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    z_axis_scale_ = this->get_parameter("z_axis_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();

    // Load mapping params
    auto joy_translation_axes_param =
        this->get_parameter("joy_translation_axes").as_integer_array();
    mapping_.joy_translation_axes.assign(joy_translation_axes_param.begin(),
                                         joy_translation_axes_param.end());

    auto joy_rotation_axes_param = this->get_parameter("joy_rotation_axes").as_integer_array();
    mapping_.joy_rotation_axes.assign(joy_rotation_axes_param.begin(),
                                      joy_rotation_axes_param.end());

    mapping_.joy_translation_signs = this->get_parameter("joy_translation_signs").as_double_array();
    mapping_.joy_rotation_signs = this->get_parameter("joy_rotation_signs").as_double_array();

    auto spacenav_translation_axes_param =
        this->get_parameter("spacenav_translation_axes").as_integer_array();
    mapping_.spacenav_translation_axes.assign(spacenav_translation_axes_param.begin(),
                                              spacenav_translation_axes_param.end());

    auto spacenav_rotation_axes_param =
        this->get_parameter("spacenav_rotation_axes").as_integer_array();
    mapping_.spacenav_rotation_axes.assign(spacenav_rotation_axes_param.begin(),
                                           spacenav_rotation_axes_param.end());

    mapping_.spacenav_translation_signs =
        this->get_parameter("spacenav_translation_signs").as_double_array();
    mapping_.spacenav_rotation_signs =
        this->get_parameter("spacenav_rotation_signs").as_double_array();

    mapping_.mode_button_joy = this->get_parameter("mode_button_joy").as_int();
    mapping_.mode_button_spacenav = this->get_parameter("mode_button_spacenav").as_int();

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

    // Print mapping for debugging.
    RCLCPP_INFO(
        this->get_logger(),
        "[joy] axes trans=%d,%d,%d signs=%.1f,%.1f,%.1f rot=%d,%d,%d signs=%.1f,%.1f,%.1f btn=%d",
        mapping_.joy_translation_axes[0], mapping_.joy_translation_axes[1],
        mapping_.joy_translation_axes[2], mapping_.joy_translation_signs[0],
        mapping_.joy_translation_signs[1], mapping_.joy_translation_signs[2],
        mapping_.joy_rotation_axes[0], mapping_.joy_rotation_axes[1], mapping_.joy_rotation_axes[2],
        mapping_.joy_rotation_signs[0], mapping_.joy_rotation_signs[1],
        mapping_.joy_rotation_signs[2], mapping_.mode_button_joy);
    RCLCPP_INFO(this->get_logger(),
                "[spacenav] axes trans=%d,%d,%d signs=%.1f,%.1f,%.1f rot=%d,%d,%d "
                "signs=%.1f,%.1f,%.1f btn=%d",
                mapping_.spacenav_translation_axes[0], mapping_.spacenav_translation_axes[1],
                mapping_.spacenav_translation_axes[2], mapping_.spacenav_translation_signs[0],
                mapping_.spacenav_translation_signs[1], mapping_.spacenav_translation_signs[2],
                mapping_.spacenav_rotation_axes[0], mapping_.spacenav_rotation_axes[1],
                mapping_.spacenav_rotation_axes[2], mapping_.spacenav_rotation_signs[0],
                mapping_.spacenav_rotation_signs[1], mapping_.spacenav_rotation_signs[2],
                mapping_.mode_button_spacenav);

    // Create a subscription to the SpaceMouse Joy messages on "/spacenav/joy".
    spacenav_joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/spacenav/joy", 10, std::bind(&JoystickInput::joyCallback, this, std::placeholders::_1));

    // Create a subscription to the 3D joystick Joy messages on "/joy".
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickInput::joy3dCallback, this, std::placeholders::_1));

    // Publisher for custom msg : Twist + teleop mode + gripper command
    teleop_cmd_publisher_ =
        this->create_publisher<extender_msgs::msg::TeleopCommand>("/teleop_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Joystick Controller node initialized");
  }

  // Callback for /joy (3D joystick)
  void JoystickInput::joy3dCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    typedef extender_msgs::msg::TeleopCommand Mode;

    extender_msgs::msg::TeleopCommand cmd_msg;

    // Update current button states from message (joystick mode button)
    if (mapping_.mode_button_joy >= 0 &&
        static_cast<size_t>(mapping_.mode_button_joy) < msg->buttons.size())
      cur_button_1_ = msg->buttons[mapping_.mode_button_joy];
    else
      cur_button_1_ = 0;

    // --- Mode switch: rising edge (single event per press) ---
    if (cur_button_1_ == 1 && last_button_1_ == 0)
    {
      if (allow_full_mode_)
      {
        // TRANSLATION_ROTATION → ROTATION → TRANSLATION → BOTH → TRANSLATION_ROTATION ...
        switch (current_mode_)
        {
        case Mode::TRANSLATION_ROTATION:
          current_mode_ = Mode::ROTATION;
          mode_str_ = "ROTATION";
          break;
        case Mode::ROTATION:
          current_mode_ = Mode::TRANSLATION;
          mode_str_ = "TRANSLATION";
          break;
        case Mode::TRANSLATION:
          current_mode_ = Mode::BOTH;
          mode_str_ = "BOTH";
          break;
        case Mode::BOTH:
          current_mode_ = Mode::TRANSLATION_ROTATION;
          mode_str_ = "TRANSLATION_ROTATION";
          break;
        }
      }
      else
      {
        current_mode_ = (current_mode_ == Mode::TRANSLATION_ROTATION) ? Mode::ROTATION
                                                                      : Mode::TRANSLATION_ROTATION;
        mode_str_ = (current_mode_ == Mode::TRANSLATION_ROTATION) ? "TRANSLATION_ROTATION"
                                                                    : "ROTATION";
      }
      RCLCPP_INFO(this->get_logger(), "[joy] Mode switched to %s", mode_str_.c_str());
    }

    last_button_1_ = cur_button_1_;

    auto twist = geometry_msgs::msg::Twist();
    if (!msg->axes.empty())
    {
      // Helper: read an axis from the Joy message
      auto getAxisValue = [&msg](int axis_index) -> double {
        if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size())
        {
          return 0.0;
        }
        return msg->axes[axis_index];
      };

      // Helper: compute one twist component from axis index + sign + scale
      auto computeComponent = [&](const std::vector<int> &axis_indices,
                                  const std::vector<double> &axis_signs, size_t component_index,
                                  double scale_factor) -> double {
        if (component_index >= axis_indices.size() || component_index >= axis_signs.size())
        {
          return 0.0;
        }
        const int axis_index = axis_indices[component_index];
        const double sign = axis_signs[component_index];
        return scale_factor * sign * getAxisValue(axis_index);
      };

      const auto &translation_axes = mapping_.joy_translation_axes;
      const auto &translation_signs = mapping_.joy_translation_signs;
      const auto &rotation_axes = mapping_.joy_rotation_axes;
      const auto &rotation_signs = mapping_.joy_rotation_signs;

      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
        twist.linear.x = computeComponent(translation_axes, translation_signs, 0, linear_scale_);
        twist.linear.y = computeComponent(translation_axes, translation_signs, 1, linear_scale_);
        twist.linear.z = computeComponent(translation_axes, translation_signs, 2, z_axis_scale_);
        break;
      case Mode::ROTATION:
        twist.angular.x = computeComponent(rotation_axes, rotation_signs, 0, angular_scale_);
        twist.angular.y = computeComponent(rotation_axes, rotation_signs, 1, angular_scale_);
        twist.angular.z = computeComponent(rotation_axes, rotation_signs, 2, angular_scale_);
        break;
      case Mode::TRANSLATION:
        twist.linear.x = computeComponent(translation_axes, translation_signs, 0, linear_scale_);
        twist.linear.y = computeComponent(translation_axes, translation_signs, 1, linear_scale_);
        twist.linear.z = computeComponent(translation_axes, translation_signs, 2, z_axis_scale_);
        break;
      case Mode::BOTH:
        twist.linear.x = computeComponent(translation_axes, translation_signs, 0, linear_scale_);
        twist.linear.y = computeComponent(translation_axes, translation_signs, 1, linear_scale_);
        twist.linear.z = computeComponent(translation_axes, translation_signs, 2, z_axis_scale_);
        twist.angular.x = computeComponent(rotation_axes, rotation_signs, 0, angular_scale_);
        twist.angular.y = computeComponent(rotation_axes, rotation_signs, 1, angular_scale_);
        twist.angular.z = computeComponent(rotation_axes, rotation_signs, 2, angular_scale_);
        break;
      }
    }

    cmd_msg.twist = twist;
    cmd_msg.mode = static_cast<uint8_t>(current_mode_);
    teleop_cmd_publisher_->publish(cmd_msg);
  }

  // Callback for /spacenav/joy (SpaceMouse)
  void JoystickInput::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    typedef extender_msgs::msg::TeleopCommand Mode;

    extender_msgs::msg::TeleopCommand cmd_msg;

    // Update current button states from message (spacenav mode button)
    if (mapping_.mode_button_spacenav >= 0 &&
        static_cast<size_t>(mapping_.mode_button_spacenav) < msg->buttons.size())
      cur_button_1_ = msg->buttons[mapping_.mode_button_spacenav];
    else
      cur_button_1_ = 0;

    // --- Mode switch: rising edge (single event per press) ---
    if (cur_button_1_ == 1 && last_button_1_ == 0)
    {
      if (allow_full_mode_)
      {
        // TRANSLATION_ROTATION → ROTATION → TRANSLATION → BOTH → TRANSLATION_ROTATION ...
        switch (current_mode_)
        {
        case Mode::TRANSLATION_ROTATION:
          current_mode_ = Mode::ROTATION;
          mode_str_ = "ROTATION";
          break;
        case Mode::ROTATION:
          current_mode_ = Mode::TRANSLATION;
          mode_str_ = "TRANSLATION";
          break;
        case Mode::TRANSLATION:
          current_mode_ = Mode::BOTH;
          mode_str_ = "BOTH";
          break;
        case Mode::BOTH:
          current_mode_ = Mode::TRANSLATION_ROTATION;
          mode_str_ = "TRANSLATION_ROTATION";
          break;
        }
      }
      else
      {
        current_mode_ = (current_mode_ == Mode::TRANSLATION_ROTATION) ? Mode::ROTATION
                                                                      : Mode::TRANSLATION_ROTATION;
      }

      RCLCPP_INFO(this->get_logger(), "Mode switched to %s", mode_str_.c_str());
    }

    last_button_1_ = cur_button_1_;

    auto twist = geometry_msgs::msg::Twist();
    if (!msg->axes.empty())
    {
      auto getAxisValue = [&msg](int axis_index) -> double {
        if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size())
        {
          return 0.0;
        }
        return msg->axes[axis_index];
      };

      auto computeComponent = [&](const std::vector<int> &axis_indices,
                                  const std::vector<double> &axis_signs, size_t component_index,
                                  double scale_factor) -> double {
        if (component_index >= axis_indices.size() || component_index >= axis_signs.size())
        {
          return 0.0;
        }
        const int axis_index = axis_indices[component_index];
        const double sign = axis_signs[component_index];
        return scale_factor * sign * getAxisValue(axis_index);
      };

      const auto &translation_axes = mapping_.spacenav_translation_axes;
      const auto &translation_signs = mapping_.spacenav_translation_signs;
      const auto &rotation_axes = mapping_.spacenav_rotation_axes;
      const auto &rotation_signs = mapping_.spacenav_rotation_signs;

      switch (current_mode_)
      {
      case Mode::TRANSLATION_ROTATION:
      case Mode::TRANSLATION:
        twist.linear.x = computeComponent(translation_axes, translation_signs, 0, linear_scale_);
        twist.linear.y = computeComponent(translation_axes, translation_signs, 1, linear_scale_);
        twist.linear.z = computeComponent(translation_axes, translation_signs, 2, z_axis_scale_);
        break;
      case Mode::ROTATION:
        twist.angular.x = computeComponent(rotation_axes, rotation_signs, 0, angular_scale_);
        twist.angular.y = computeComponent(rotation_axes, rotation_signs, 1, angular_scale_);
        twist.angular.z = computeComponent(rotation_axes, rotation_signs, 2, angular_scale_);
        break;
      case Mode::BOTH:
        twist.linear.x = computeComponent(translation_axes, translation_signs, 0, linear_scale_);
        twist.linear.y = computeComponent(translation_axes, translation_signs, 1, linear_scale_);
        twist.linear.z = computeComponent(translation_axes, translation_signs, 2, z_axis_scale_);
        twist.angular.x = computeComponent(rotation_axes, rotation_signs, 0, angular_scale_);
        twist.angular.y = computeComponent(rotation_axes, rotation_signs, 1, angular_scale_);
        twist.angular.z = computeComponent(rotation_axes, rotation_signs, 2, angular_scale_);
        break;
      }
    }

    cmd_msg.twist = twist;
    cmd_msg.mode = static_cast<uint8_t>(current_mode_);
    teleop_cmd_publisher_->publish(cmd_msg);
  }
} // namespace input_interfaces
