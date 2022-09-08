/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef WOLF_HW_SIM_H
#define WOLF_HW_SIM_H

// ROS includes
#include <ros/ros.h>
#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>

// Gazebo and ignition includes
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

// Gazebo ros control include
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF include
#include <urdf/model.h>

// HW interface
#include <wolf_hardware_interface/wolf_robot_hw.h>

namespace wolf_gazebo_interface
{

/**
 * @class This class implements the Gazebo hardware interface for a generic robot to control through a whole body controller
 * @brief This hardware interface is loaded to Gazebo using gazebo_ros_control plugin
 * which required the initSim, readSim and writeSim methods override in this class.
 */
class WolfRobotHwSim : public gazebo_ros_control::RobotHWSim, public hardware_interface::WolfRobotHwInterface
{
public:

  typedef std::shared_ptr<WolfRobotHwSim> Ptr;

  const std::string CLASS_NAME = "WolfRobotHwSim";

  /**
     * @brief Initializes the hardware interface by reading the urdf file
     * @param const std::string& robot_namespace Robot namespace
     * @param ros::NodeHandle Model node handle
     * @param gazebo::physics::ModelPtr Gazebo model pointer
     * @param const urdf::Model *const URDF model
     * @param std::vector<transmission_interface::TransmissionInfo> Transmissions information
     */
  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model,
               const urdf::Model *const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);

  /**
     * @brief Reads all the sensors from Gazebo: encoders and imu
     * @param ros::Time Simulated time
     * @param ros::Duration Simulated period
     */
  void readSim(ros::Time time, ros::Duration period);

  /**
     * @brief Writes the forces values to Gazebo
     * @param ros::Time Simulated time
     * @param ros::Duration Simulated period
     */
  void writeSim(ros::Time time, ros::Duration period);

  /**
     * @brief Freeze Base
     */
  bool freezeBase(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:

  std::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor_;
  std::map<std::string,std::shared_ptr<gazebo::sensors::ContactSensor> > contact_sensors_;
  std::vector<std::string> contact_names_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr sim_model_;

  ros::ServiceServer ss_;
  bool freeze_base_sim_;

  ignition::math::Pose3d inital_pose_;
  ignition::math::Matrix3d base_R_world_;
  ignition::math::Vector3d vector3d_tmp_;
  ignition::math::Vector3d vector3d_tmp2_;
  ignition::math::Quaterniond quaterniond_tmp_;
  ignition::math::Pose3d link_pose_;
  ignition::math::Vector3d world_force_;
  ignition::math::Vector3d local_force_;
};

}

#endif
