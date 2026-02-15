#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>

namespace l298n_hardware
{

class L298NSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(L298NSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // GPIO pins
  int left_pwm_{-1}, left_in1_{-1}, left_in2_{-1};
  int right_pwm_{-1}, right_in1_{-1}, right_in2_{-1};

  // params
  double max_wheel_rad_s_{11.0};   // подстрой под моторы
  int pwm_max_{255};              // pigpio PWM: 0..255

  // command/state
  double left_cmd_{0.0}, right_cmd_{0.0};
  double left_state_{0.0}, right_state_{0.0};

  void set_motor(double vel_rad_s, int pwm, int in1, int in2);
};

}  // namespace l298n_hardware
