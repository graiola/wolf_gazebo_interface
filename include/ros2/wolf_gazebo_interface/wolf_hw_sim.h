#ifndef WOLF_HW_SIM_H
#define WOLF_HW_SIM_H

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

// Gazebo and Ignition includes
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

// URDF include
#include <urdf/model.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

// HW interface
#include <wolf_hardware_interface/wolf_robot_hw.h>

namespace wolf_gazebo_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class WolfRobotHwSim : public gazebo_ros2_control::GazeboSystemInterface, public hardware_interface::WolfRobotHwInterface
{
public:
  using Ptr = std::shared_ptr<WolfRobotHwSim>;

  const std::string CLASS_NAME = "WolfRobotHwSim";

  // Initialize the system interface
  virtual bool initSim(
      rclcpp::Node::SharedPtr & model_nh,
      gazebo::physics::ModelPtr parent_model,
      const hardware_interface::HardwareInfo & hardware_info,
      sdf::ElementPtr sdf);

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:

  // Gazebo simulation variables
  std::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor_;
  std::map<std::string, std::shared_ptr<gazebo::sensors::ContactSensor>> contact_sensors_;
  std::vector<std::string> contact_names_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;     // Vector of joints in the Gazebo simulation
  gazebo::physics::ModelPtr sim_model_;                   // Pointer to the Gazebo simulation model

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ss_;   // ROS 2 service for freezing the base
  bool freeze_base_sim_;                                  // Boolean to control base freezing

  ignition::math::Pose3d inital_pose_;                    // Initial pose of the robot in Gazebo
  ignition::math::Matrix3d base_R_world_;                 // Rotation matrix for the base in the world frame
  ignition::math::Vector3d vector3d_tmp_;                 // Temporary vector for calculations
  ignition::math::Vector3d vector3d_tmp2_;                // Additional temporary vector for calculations
  ignition::math::Quaterniond quaterniond_tmp_;           // Temporary quaternion for calculations
};

}  // namespace wolf_gazebo_interface

#endif  // WOLF_HW_SIM_H
