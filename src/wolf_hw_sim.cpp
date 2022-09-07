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

#include <wolf_gazebo_interface/wolf_hw_sim.h>

#include <gazebo/sensors/SensorManager.hh>
#include <ignition/math/Vector3.hh>

PLUGINLIB_EXPORT_CLASS(wolf_gazebo_interface::WolfRobotHwSim, gazebo_ros_control::RobotHWSim)

namespace wolf_gazebo_interface
{

  using namespace hardware_interface;

  bool WolfRobotHwSim::initSim(const std::string& /*robot_namespace*/,
                              ros::NodeHandle model_nh,
                              gazebo::physics::ModelPtr parent_model,
                              const urdf::Model *const /*urdf_model*/,
                              std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    const auto& sensor_manager  = gazebo::sensors::SensorManager::Instance();
    sim_model_ = parent_model;
    inital_pose_ = sim_model_->WorldPose();
    robot_name_ = sim_model_->GetName();

    // Hardware interfaces: Joints
    std::vector<std::string> joint_names(transmissions.size());
    // Initialize values from the transmission interface i.e. by using actuated joints (no floating base).
    for (unsigned int j=0; j < transmissions.size(); j++)
    {
      // Check that this transmission has one joint
      if (transmissions[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has no associated joints.");
        continue;
      }
      else if (transmissions[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has more than one joint. Currently the default robot hardware simulation "
                              << " interface only supports one.");
        continue;
      }
      // Check that this transmission has one actuator
      if (transmissions[j].actuators_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has no associated actuators.");
        continue;
      }
      else if (transmissions[j].actuators_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Transmission " << transmissions[j].name_
                              << " has more than one actuator. Currently the default robot hardware simulation "
                              << " interface only supports one.");
        continue;
      }
      joint_names[j] = transmissions[j].joints_[0].name_;
    }
    WolfRobotHwInterface::initializeJointsInterface(joint_names);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_effort_interface_);
    for(unsigned int j=0;j<n_dof_;j++)
    {
      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
      if (!joint)
      {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"This robot has a joint named \"" << joint_names_[j]
                               << "\" which is not in the gazebo model.");
        return false;
      }
      // Set limits
      sim_joints_.push_back(joint);
      joint_effort_limits_[j] = joint->GetEffortLimit(0);
      // joint->SetEffortLimit(0,joint_effort_limits_[j]);
    }

    // Hardware interfaces: IMU sensor
    WolfRobotHwInterface::initializeImuInterface(loadImuLinkNameFromSRDF());
    imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor_manager->GetSensor(imu_data_.frame_id));
    if(!this->imu_sensor_)
        ROS_WARN_NAMED(CLASS_NAME,"Could not find base IMU sensor in gazebo, using the ground truth to fill the IMU data instead.");
    registerInterface(&imu_sensor_interface_);

    // Hardware interfaces: Ground Truth
    WolfRobotHwInterface::initializeGroundTruthInterface(loadBaseLinkNameFromSRDF());
    registerInterface(&ground_truth_interface_);

    // Freeze base service
    ss_ = model_nh.advertiseService("freeze_base", &WolfRobotHwSim::freezeBase, this);
    freeze_base_sim_ = false;

    return true;
  }

  bool WolfRobotHwSim::freezeBase(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    // Freeze base toggle
    freeze_base_sim_ = !freeze_base_sim_;
    if(freeze_base_sim_)
    {
      ROS_INFO_NAMED(CLASS_NAME,"Freeze Base on!");
      sim_model_->SetWorldPose(inital_pose_);
    }
    else
      ROS_INFO_NAMED(CLASS_NAME,"Freeze Base off!");

    sim_model_->SetGravityMode(!freeze_base_sim_);

    return true;
  }

  void WolfRobotHwSim::readSim(ros::Time /*time*/, ros::Duration period)
  {

    for (unsigned int j=0; j < n_dof_; j++)
    {
      if (joint_types_[j] == urdf::Joint::PRISMATIC)
        joint_position_[j] = sim_joints_[j]->Position(0);

      else
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                                sim_joints_[j]->Position(0));

      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce(static_cast<unsigned int>(0));
    }

    //Ground truth:
    ignition::math::Vector3d  gzLinearVel = sim_model_->WorldLinearVel();
    base_lin_vel_[0] = gzLinearVel.X();
    base_lin_vel_[1] = gzLinearVel.Y();
    base_lin_vel_[2] = gzLinearVel.Z();

    base_lin_acc_[0] = (base_lin_vel_[0] - base_lin_vel_prev_[0])/period.toSec();
    base_lin_acc_[1] = (base_lin_vel_[1] - base_lin_vel_prev_[1])/period.toSec();
    base_lin_acc_[2] = (base_lin_vel_[2] - base_lin_vel_prev_[2])/period.toSec();
    base_lin_vel_prev_[0] = base_lin_vel_[0];
    base_lin_vel_prev_[1] = base_lin_vel_[1];
    base_lin_vel_prev_[2] = base_lin_vel_[2];

    ignition::math::Vector3d  gzAngularVel = sim_model_->WorldAngularVel();
    base_ang_vel_[0] = gzAngularVel.X();
    base_ang_vel_[1] = gzAngularVel.Y();
    base_ang_vel_[2] = gzAngularVel.Z();

    base_ang_acc_[0] = (base_ang_vel_[0] - base_ang_vel_prev_[0])/period.toSec();
    base_ang_acc_[1] = (base_ang_vel_[1] - base_ang_vel_prev_[1])/period.toSec();
    base_ang_acc_[2] = (base_ang_vel_[2] - base_ang_vel_prev_[2])/period.toSec();
    base_ang_vel_prev_[0] = base_ang_vel_[0];
    base_ang_vel_prev_[1] = base_ang_vel_[1];
    base_ang_vel_prev_[2] = base_ang_vel_[2];

    ignition::math::Pose3d gzPose = sim_model_->WorldPose();
    base_lin_pos_[0] = gzPose.Pos().X();
    base_lin_pos_[1] = gzPose.Pos().Y();
    base_lin_pos_[2] = gzPose.Pos().Z();
    base_orientation_[0] = gzPose.Rot().W();
    base_orientation_[1] = gzPose.Rot().X();
    base_orientation_[2] = gzPose.Rot().Y();
    base_orientation_[3] = gzPose.Rot().Z();

    // IMU data:
    ignition::math::Quaterniond imu_quat(1, 0, 0, 0);
    ignition::math::Vector3d imu_ang_vel(0, 0, 0);
    ignition::math::Vector3d imu_lin_acc(0, 0, 0);

    // In this case we are using the IMU sensor which has angular velocities and accelerations defined wrt the trunk/base
    if(this->imu_sensor_ != nullptr)
    {
      imu_quat    = imu_sensor_->Orientation();
      imu_ang_vel = imu_sensor_->AngularVelocity();
      imu_lin_acc = imu_sensor_->LinearAcceleration();
    }
    // In this case we need to transform the angular velocities and accelerations from world measurements (Gazebo) to trunk/base (IMU)
    else
    {

      imu_data_.frame_id = gt_data_.frame_id;

      // Orientation
      quaterniond_tmp_.w() = gzPose.Rot().W();
      quaterniond_tmp_.x() = gzPose.Rot().X();
      quaterniond_tmp_.y() = gzPose.Rot().Y();
      quaterniond_tmp_.z() = gzPose.Rot().Z();
      quaterniond_tmp_.normalize();
      world_R_base_ = quaterniond_tmp_.toRotationMatrix();
      imu_quat.W() = gzPose.Rot().W();
      imu_quat.X() = gzPose.Rot().X();
      imu_quat.Y() = gzPose.Rot().Y();
      imu_quat.Z() = gzPose.Rot().Z();

      // Angular velocities
      vector3d_tmp_ << static_cast<double>(gzAngularVel.X()),
                       static_cast<double>(gzAngularVel.Y()),
                       static_cast<double>(gzAngularVel.Z());
      vector3d_tmp_ = world_R_base_.transpose() * vector3d_tmp_; // base_angular_vel = base_R_world * world_angular_vel
      imu_ang_vel.X() = vector3d_tmp_(0);
      imu_ang_vel.Y() = vector3d_tmp_(1);
      imu_ang_vel.Z() = vector3d_tmp_(2);

      // Linear accelerations
      vector3d_tmp_ << static_cast<double>(base_lin_acc_[0]),
                       static_cast<double>(base_lin_acc_[1]),
                       static_cast<double>(base_lin_acc_[2]);
      vector3d_tmp_ = world_R_base_.transpose() * vector3d_tmp_; // base_linear_acc = base_R_world * world_linear_acc
      imu_lin_acc.X() = vector3d_tmp_(0);
      imu_lin_acc.Y() = vector3d_tmp_(1);
      imu_lin_acc.Z() = vector3d_tmp_(2);
    }

    imu_orientation_[0] = imu_quat.W();
    imu_orientation_[1] = imu_quat.X();
    imu_orientation_[2] = imu_quat.Y();
    imu_orientation_[3] = imu_quat.Z();

    imu_ang_vel_[0] = imu_ang_vel.X();
    imu_ang_vel_[1] = imu_ang_vel.Y();
    imu_ang_vel_[2] = imu_ang_vel.Z();

    imu_lin_acc_[0] = imu_lin_acc.X();
    imu_lin_acc_[1] = imu_lin_acc.Y();
    imu_lin_acc_[2] = imu_lin_acc.Z();
  }


  void WolfRobotHwSim::writeSim(ros::Time /*time*/, ros::Duration /*period*/)
  {

    if(freeze_base_sim_)
    {
      sim_model_->SetWorldPose(inital_pose_);
      gazebo::physics::LinkPtr base_link = sim_model_->GetLink(gt_data_.frame_id);
      if(base_link != nullptr)
      {
        //Set velocities and accelerations only for the base link:
        base_link->SetLinearVel(ignition::math::Vector3d::Zero);
        //base_link->SetLinearAccel(ignition::math::Vector3d::Zero); // Deprecated in gazebo9
        base_link->SetAngularVel(ignition::math::Vector3d::Zero);
        //base_link->SetAngularAccel(ignition::math::Vector3d::Zero); // Deprecated in gazebo9
      }
    }

    for (unsigned int j=0; j < n_dof_; j++)
      sim_joints_[j]->SetForce(0, joint_effort_command_[j]);
  }

} // namespace
