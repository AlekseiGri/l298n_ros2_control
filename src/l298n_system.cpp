#include "l298n_hardware/l298n_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

// pigpio
#include <pigpio.h>

namespace l298n_hardware
{

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
    RCLCPP_ERROR(rclcpp::get_logger("L298NSystem"), "Param parse error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
L298NSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &left_state_);
  states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_state_);
  return states;
}

std::vector<hardware_interface::CommandInterface>
L298NSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &left_cmd_);
  cmds.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_cmd_);
  return cmds;
}

hardware_interface::CallbackReturn L298NSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (gpioInitialise() < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("L298NSystem"), "pigpio init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  gpioSetMode(left_pwm_,  PI_OUTPUT);
  gpioSetMode(left_in1_,  PI_OUTPUT);
  gpioSetMode(left_in2_,  PI_OUTPUT);

  gpioSetMode(right_pwm_, PI_OUTPUT);
  gpioSetMode(right_in1_, PI_OUTPUT);
  gpioSetMode(right_in2_, PI_OUTPUT);

  // стоп по умолчанию
  gpioPWM(left_pwm_,  0);
  gpioPWM(right_pwm_, 0);
  gpioWrite(left_in1_, 0);
  gpioWrite(left_in2_, 0);
  gpioWrite(right_in1_, 0);
  gpioWrite(right_in2_, 0);

  RCLCPP_INFO(rclcpp::get_logger("L298NSystem"), "L298N hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn L298NSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  gpioPWM(left_pwm_,  0);
  gpioPWM(right_pwm_, 0);

  gpioWrite(left_in1_, 0);
  gpioWrite(left_in2_, 0);
  gpioWrite(right_in1_, 0);
  gpioWrite(right_in2_, 0);
  
  gpioTerminate();
  RCLCPP_INFO(rclcpp::get_logger("L298NSystem"), "L298N hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void L298NSystem::set_motor(double vel_rad_s, int pwm, int in1, int in2)
{
  bool forward = vel_rad_s >= 0.0;
  double norm = 0.0;

  if (max_wheel_rad_s_ > 0.0) {
    norm = std::clamp(std::abs(vel_rad_s) / max_wheel_rad_s_, 0.0, 1.0);
  }

  gpioWrite(in1, forward ? 1 : 0);
  gpioWrite(in2, forward ? 0 : 1);

  int duty = static_cast<int>(norm * pwm_max_);
  gpioPWM(pwm, duty);
}

hardware_interface::return_type L298NSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  set_motor(left_cmd_,  left_pwm_,  left_in1_,  left_in2_);
  set_motor(right_cmd_, right_pwm_, right_in1_, right_in2_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type L298NSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Без энкодеров: считаем, что достигли команды
  left_state_  = left_cmd_;
  right_state_ = right_cmd_;
  return hardware_interface::return_type::OK;
}

}  // namespace l298n_hardware

PLUGINLIB_EXPORT_CLASS(
  l298n_hardware::L298NSystem,
  hardware_interface::SystemInterface
)
