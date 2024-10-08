#include <wolf_gazebo_interface/wolf_hw_sim.h>
#include <gazebo/sensors/SensorManager.hh>

namespace wolf_gazebo_interface
{

using namespace hardware_interface;

bool WolfRobotHwSim::initSim(
    rclcpp::Node::SharedPtr& model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf)
{

  nh_ = model_nh;

  RCLCPP_INFO_STREAM(nh_->get_logger(),"InitSim");

  const auto& sensor_manager  = gazebo::sensors::SensorManager::Instance();
  sim_model_ = parent_model;
  inital_pose_ = sim_model_->WorldPose();
  robot_name_ = sim_model_->GetName();

  // Hardware interfaces: Joints
  std::vector<std::string> joint_names(hardware_info.joints.size());
  for (unsigned int j=0; j < joint_names.size(); j++)
    joint_names_[j] = hardware_info.joints[j].name;

  WolfRobotHwInterface::parseSRDF(robot_name_);

  // register the state handles
  WolfRobotHwInterface::initializeJointsInterface(joint_names_);

  for(unsigned int j=0;j<n_dof_;j++)
  {
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(),"This robot has a joint named \"" << joint_names_[j]
                          << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);
    // Set limits
    //joint_effort_limits_[j] = joint->GetEffortLimit(0);
    //joint->SetEffortLimit(0,joint_effort_limits_[j]);
  }

  // Hardware interfaces: IMU sensor
  WolfRobotHwInterface::initializeImuInterface(loadImuLinkNameFromSRDF());
  imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor_manager->GetSensor(imu_data_.header.frame_id));
  if(!imu_sensor_)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),"Could not find base IMU sensor in gazebo.");
    return false;
  }

  return true;
}

std::vector<StateInterface> WolfRobotHwSim::export_state_interfaces()
{
  return WolfRobotHwInterface::exportStateInterfaces();
}

std::vector<CommandInterface> WolfRobotHwSim::export_command_interfaces()
{
  return WolfRobotHwInterface::exportCommandInterfaces();
}

return_type WolfRobotHwSim::configure(const hardware_interface::HardwareInfo & system_info)
{
  if (configure_default(system_info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

return_type WolfRobotHwSim::start()
{
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

return_type WolfRobotHwSim::stop()
{
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

return_type WolfRobotHwSim::read()
{
  for (unsigned int j = 0; j < n_dof_; j++) {
    if (sim_joints_[j]) {
      joint_position_[j] = sim_joints_[j]->Position(0);
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j]   = sim_joints_[j]->GetForce(0u);
    }
  }

  if(imu_sensor_ != nullptr) {

    imu_orientation_[0] = imu_sensor_->Orientation().X();
    imu_orientation_[1] = imu_sensor_->Orientation().Y();
    imu_orientation_[2] = imu_sensor_->Orientation().Z();
    imu_orientation_[3] = imu_sensor_->Orientation().W();

    imu_ang_vel_[0] = imu_sensor_->AngularVelocity().X();
    imu_ang_vel_[1] = imu_sensor_->AngularVelocity().Y();
    imu_ang_vel_[2] = imu_sensor_->AngularVelocity().Z();

    imu_lin_acc_[0] = imu_sensor_->LinearAcceleration().X();
    imu_lin_acc_[1] = imu_sensor_->LinearAcceleration().Y();
    imu_lin_acc_[2] = imu_sensor_->LinearAcceleration().Z();
  }

  return hardware_interface::return_type::OK;
}

return_type WolfRobotHwSim::write()
{
  for (unsigned int j=0; j < n_dof_; j++)
    sim_joints_[j]->SetForce(0, joint_effort_command_[j]);
  return hardware_interface::return_type::OK;
}

}  // namespace wolf_gazebo_interface

// This macro is required to register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(wolf_gazebo_interface::WolfRobotHwSim, hardware_interface::SystemInterface)
