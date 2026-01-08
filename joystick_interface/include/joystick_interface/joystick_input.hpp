#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "extender_msgs/msg/teleop_command.hpp"

namespace input_interfaces
{
  /// @brief Struct holding axis and button mapping configuration.
  struct JoystickMapping
  {
    // /joy (3D joystick) mapping
    std::vector<int> joy_translation_axes{1, 0, 2};
    std::vector<int> joy_rotation_axes{0, 1, 2};
    std::vector<double> joy_translation_signs{1.0, 1.0, 1.0};
    std::vector<double> joy_rotation_signs{-1.0, -1.0, -1.0};

    // /spacenav/joy (SpaceMouse) mapping
    std::vector<int> spacenav_translation_axes{4, 3, 5};
    std::vector<int> spacenav_rotation_axes{3, 4, 5};
    std::vector<double> spacenav_translation_signs{1.0, -1.0, 1.0};
    std::vector<double> spacenav_rotation_signs{1.0, -1.0, -1.0};

    // Buttons
    int mode_button_joy{11};       
    int mode_button_spacenav{1}; 

  };

  /// @brief Node that will subscribe either to a SpaceMouse or a normal joystick, and send out
  /// teloep_cmd messages (twist + mode of command) on the topic /teleop_cmd
  class JoystickInput : public rclcpp::Node
  {
  public:
    /// @brief Constructor of the class. Will intialize parameters as well a subscriber and
    /// publisher.
    JoystickInput();

  private:
    /// @brief The current teleoperation mode. Defaults to TRANSLATION_ROTATION.
    uint8_t current_mode_{extender_msgs::msg::TeleopCommand::TRANSLATION_ROTATION};

    /// @brief A configuration parameter to enable or disable access to the TRANSLATION or BOTH
    /// modes.
    bool allow_full_mode_{false};

    std::string mode_str_;

    /// @brief Configuration mapping for axes and buttons.
    JoystickMapping mapping_{};

    /// @brief Callback function to process incoming Joy messages from the SpaceMouse.
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief Callback function to process incoming Joy messages from the 3D joystick.
    void joy3dCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief Scaling factor for linear velocity commands (X, Y axes).
    double linear_scale_;
    /// @brief Scaling factor for linear velocity commands on the Z-axis.
    double z_axis_scale_;
    /// @brief Scaling factor for angular velocity commands.
    double angular_scale_;

    /// @brief Subscriber for receiving Joy messages from the SpaceMouse on the `/spacenav/joy`
    /// topic.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr spacenav_joy_subscriber_;

    /// @brief Subscriber for receiving Joy messages from a standard joystick on the `/joy` topic.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    /// @brief Publisher for the custom `TeleopCmd` message on the `/teleop_cmd` topic.
    rclcpp::Publisher<extender_msgs::msg::TeleopCommand>::SharedPtr teleop_cmd_publisher_;

    // Button state handling variables to detect single presses
    int last_button_1_{0}; ///< Stores the previous state of button 1 (mode toggle).
    int cur_button_1_{0};  ///< Stores the current state of button 1.
    bool button_1_already_handled_ = false; ///< Flag to ensure button 1 press is handled only once.
  };
} // namespace control_interfaces
