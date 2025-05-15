#include <mujoco_ros2_control/mujoco_ros2_control_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_accessories
{

class RidgebackSystemInterface : public mujoco_ros2_control::MjSimSystemInterface
{
public:
  // Internal buffers
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<std::string> joint_names_;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;

    joint_names_.resize(info.joints.size());
    hw_commands_.resize(info.joints.size(), 0.0);
    hw_positions_.resize(info.joints.size(), 0.0);
    hw_velocities_.resize(info.joints.size(), 0.0);

    for (size_t i = 0; i < info.joints.size(); ++i)
    {
      joint_names_[i] = info.joints[i].name;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      int id = joint_name_to_mj_id_.at(joint_names_[i]);
      hw_positions_[i] = data_->qpos[id];
      hw_velocities_[i] = data_->qvel[id];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      int id = joint_name_to_actuator_id_.at(joint_names_[i]);
      data_->ctrl[id] = hw_commands_[i];
    }
    return hardware_interface::return_type::OK;
  }
};

}  // namespace mujoco_accessories

// Register plugin
PLUGINLIB_EXPORT_CLASS(mujoco_accessories::RidgebackSystemInterface, hardware_interface::SystemInterface)

