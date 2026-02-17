#include "l298n_hardware/l298n_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

// pigpio
#include <pigpiod_if2.h>

namespace l298n_hardware
{
  // Initialize static members
  hardware_interface::CallbackReturn L298NSystem::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      left_pwm_  = std::stoi(info.hardware_parameters.at("left_pwm_pin"));
      left_in1_  = std::stoi(info.hardware_parameters.at("left_dir1_pin"));
      left_in2_  = std::stoi(info.hardware_parameters.at("left_dir2_pin"));

      right_pwm_ = std::stoi(info.hardware_parameters.at("right_pwm_pin"));
      right_in1_ = std::stoi(info.hardware_parameters.at("right_dir1_pin"));
      right_in2_ = std::stoi(info.hardware_parameters.at("right_dir2_pin"));

      if (info.hardware_parameters.count("max_wheel_rad_s")) {
        max_wheel_rad_s_ = std::stod(info.hardware_parameters.at("max_wheel_rad_s"));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("L298NSystem"),
        "Parameter parsing failed: %s", e.what()
      );

      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Export state interfaces
  std::vector<hardware_interface::StateInterface>
  L298NSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> states;
    states.emplace_back(
      "left_wheel_joint",
      hardware_interface::HW_IF_POSITION,
      &left_pos_);

    states.emplace_back(
      "left_wheel_joint",
      hardware_interface::HW_IF_VELOCITY,
      &left_state_);

    states.emplace_back(
      "right_wheel_joint",
      hardware_interface::HW_IF_POSITION,
      &right_pos_);

    states.emplace_back(
      "right_wheel_joint",
      hardware_interface::HW_IF_VELOCITY,
      &right_state_);

    return states;
  }

  // Export command interfaces
  std::vector<hardware_interface::CommandInterface>
  L298NSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> cmds;
    cmds.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &left_cmd_);
    cmds.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_cmd_);
    return cmds;
  }

  // Lifecycle management
  hardware_interface::CallbackReturn L298NSystem::on_activate(
    const rclcpp_lifecycle::State &)
  {
    pi_ = pigpio_start(NULL, NULL);
    if (pi_ < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("L298NSystem"),
        "pigpio daemon connection failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // setting GPIO modes
    if (set_mode(pi_, left_pwm_, PI_OUTPUT) < 0 ||
        set_mode(pi_, left_in1_, PI_OUTPUT) < 0 ||
        set_mode(pi_, left_in2_, PI_OUTPUT) < 0 ||
        set_mode(pi_, right_pwm_, PI_OUTPUT) < 0 ||
        set_mode(pi_, right_in1_, PI_OUTPUT) < 0 ||
        set_mode(pi_, right_in2_, PI_OUTPUT) < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("L298NSystem"),
        "Failed to configure GPIO modes");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // setting PWM frequency and range

    set_PWM_frequency(pi_, left_pwm_, 20000);
    set_PWM_frequency(pi_, right_pwm_, 20000);

    set_PWM_range(pi_, left_pwm_,  pwm_max_);
    set_PWM_range(pi_, right_pwm_, pwm_max_);

    // initialize motors to stopped
    set_PWM_dutycycle(pi_, left_pwm_,  0);
    set_PWM_dutycycle(pi_, right_pwm_, 0);

    gpio_write(pi_, left_in1_, 0);
    gpio_write(pi_, left_in2_, 0);
    gpio_write(pi_, right_in1_, 0);
    gpio_write(pi_, right_in2_, 0);

    RCLCPP_INFO(
      rclcpp::get_logger("L298NSystem"),
      "L298N hardware activated (daemon mode)");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // deactivate: stop motors and release pigpio resources
  hardware_interface::CallbackReturn L298NSystem::on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    if (pi_ >= 0)
    {
      set_PWM_dutycycle(pi_, left_pwm_,  0);
      set_PWM_dutycycle(pi_, right_pwm_, 0);

      gpio_write(pi_, left_in1_, 0);
      gpio_write(pi_, left_in2_, 0);
      gpio_write(pi_, right_in1_, 0);
      gpio_write(pi_, right_in2_, 0);

      pigpio_stop(pi_);
      pi_ = -1;
    }

  RCLCPP_INFO(rclcpp::get_logger("L298NSystem"),
              "L298N hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Helper to set motor direction and speed
  void L298NSystem::set_motor(double vel_rad_s, int pwm, int in1, int in2)
  {
    if (pi_ < 0)
    return;

    bool forward = vel_rad_s >= 0.0;
    double norm = 0.0;

    if (max_wheel_rad_s_ > 0.0) {
      norm = std::clamp(std::abs(vel_rad_s) / max_wheel_rad_s_, 0.0, 1.0);
    }

    gpio_write(pi_, in1, forward ? 1 : 0);
    gpio_write(pi_, in2, forward ? 0 : 1);

    int duty = static_cast<int>(norm * pwm_max_);
    set_PWM_dutycycle(pi_, pwm, duty);
  }

  // Write commands to hardware
  hardware_interface::return_type L298NSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (pi_ < 0)
      return hardware_interface::return_type::ERROR;

    RCLCPP_INFO(logger_,
              "WRITE left_cmd: %.3f  right_cmd: %.3f",
              left_cmd_, right_cmd_);

    set_motor(left_cmd_,  left_pwm_,  left_in1_,  left_in2_);
    set_motor(right_cmd_, right_pwm_, right_in1_, right_in2_);
    return hardware_interface::return_type::OK;
  }

  // Simulate state updates based on commands (for testing without real hardware)
  hardware_interface::return_type L298NSystem::read(
    const rclcpp::Time &, const rclcpp::Duration & period)
  {
    double dt = period.seconds();

    left_state_  = left_cmd_;
    right_state_ = right_cmd_;

    left_pos_  += left_state_  * dt;
    right_pos_ += right_state_ * dt;

    return hardware_interface::return_type::OK;
  }
  
}  // namespace l298n_hardware

// Export the system as a plugin
PLUGINLIB_EXPORT_CLASS(
  l298n_hardware::L298NSystem,
  hardware_interface::SystemInterface
)
