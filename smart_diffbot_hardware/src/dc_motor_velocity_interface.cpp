#include "smart_diffbot_hardware/dc_motor_velocity_interface.hpp"


namespace smart_diffbot_hardware_interface
{

  // INIT
  hardware_interface::CallbackReturn DCMotorVelocityInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    // Basic checks
    if (
      hardware_interface::ActuatorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const hardware_interface::ComponentInfo & joint = info_.joints[0];

    // DC Motor has only one interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // And it should be velocity interface
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Init command and state
    hw_joint_command_ = 0.0;
    hw_joint_position_ = 0.0;
    hw_joint_velocity_ = 0.0;

    // All is well
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // WRITE
  hardware_interface::return_type DCMotorVelocityInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // Dummy 
    (void)time;
    (void)period;

    return hardware_interface::return_type::OK;
  }

  // READ
  hardware_interface::return_type DCMotorVelocityInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // Dummy
    (void)time;
    (void)period;

    return hardware_interface::return_type::OK;
  }

  // STATE INTERFACE
  std::vector<hardware_interface::StateInterface> DCMotorVelocityInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_joint_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocity_));
  
    return state_interfaces;
  }

  // COMMAND INTERFACE
  std::vector<hardware_interface::CommandInterface> DCMotorVelocityInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Velocity command interface for this actuator
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_command_));

    return command_interfaces;
  }

} // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smart_diffbot_hardware_interface::DCMotorVelocityInterface, hardware_interface::ActuatorInterface)
