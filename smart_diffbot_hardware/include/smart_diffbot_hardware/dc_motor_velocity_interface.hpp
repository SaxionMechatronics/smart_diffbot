#ifndef SMART_DIFFBOT_HARDWARE__DC_MOTOR_VELOCITY_INTERFACE_HPP_
#define SMART_DIFFBOT_HARDWARE__DC_MOTOR_VELOCITY_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"


namespace smart_diffbot_hardware_interface
{
  class DCMotorVelocityInterface : public hardware_interface::ActuatorInterface
  {
  public:
    DCMotorVelocityInterface(){}

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;
   
    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;


  private:

    // Command and states
    double hw_joint_command_;
    double hw_joint_position_;
    double hw_joint_velocity_;

  }; // class


} // namespace

#endif  
