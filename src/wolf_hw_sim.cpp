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
    WolfRobotHwInterface::setNodeHandle(model_nh);
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

    // Hardware interfaces: Contact sensors
    contact_names_ = loadContactNamesFromSRDF();
    WolfRobotHwInterface::initializeContactSensorsInterface(contact_names_);
    for(unsigned int i=0; i < contact_sensor_names_.size(); i++)
    {
      contact_sensors_[contact_sensor_names_[i]] = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor_manager->GetSensor(contact_sensor_names_[i]));
      if(!this->contact_sensors_[contact_sensor_names_[i]])
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Could not find "<< contact_sensor_names_[i] <<".");
    }
    registerInterface(&contact_sensor_interface_);

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
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], sim_joints_[j]->Position(0));

      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce(static_cast<unsigned int>(0));
    }

    // Ground truth data:
    base_lin_vel_[0] = sim_model_->WorldLinearVel().X();
    base_lin_vel_[1] = sim_model_->WorldLinearVel().Y();
    base_lin_vel_[2] = sim_model_->WorldLinearVel().Z();

    base_lin_acc_[0] = sim_model_->WorldLinearAccel().X();//(base_lin_vel_[0] - base_lin_vel_prev_[0])/period.toSec();
    base_lin_acc_[1] = sim_model_->WorldLinearAccel().Y();//(base_lin_vel_[1] - base_lin_vel_prev_[1])/period.toSec();
    base_lin_acc_[2] = sim_model_->WorldLinearAccel().Z();//(base_lin_vel_[2] - base_lin_vel_prev_[2])/period.toSec();
    base_lin_vel_prev_[0] = base_lin_vel_[0];
    base_lin_vel_prev_[1] = base_lin_vel_[1];
    base_lin_vel_prev_[2] = base_lin_vel_[2];

    base_ang_vel_[0] = sim_model_->WorldAngularVel().X();
    base_ang_vel_[1] = sim_model_->WorldAngularVel().Y();
    base_ang_vel_[2] = sim_model_->WorldAngularVel().Z();

    base_ang_acc_[0] = sim_model_->WorldAngularAccel().X();//(base_ang_vel_[0] - base_ang_vel_prev_[0])/period.toSec();
    base_ang_acc_[1] = sim_model_->WorldAngularAccel().Y();//(base_ang_vel_[1] - base_ang_vel_prev_[1])/period.toSec();
    base_ang_acc_[2] = sim_model_->WorldAngularAccel().Z();//(base_ang_vel_[2] - base_ang_vel_prev_[2])/period.toSec();
    base_ang_vel_prev_[0] = base_ang_vel_[0];
    base_ang_vel_prev_[1] = base_ang_vel_[1];
    base_ang_vel_prev_[2] = base_ang_vel_[2];

    base_lin_pos_[0]     = sim_model_->WorldPose().Pos().X();
    base_lin_pos_[1]     = sim_model_->WorldPose().Pos().Y();
    base_lin_pos_[2]     = sim_model_->WorldPose().Pos().Z();
    base_orientation_[0] = sim_model_->WorldPose().Rot().W();
    base_orientation_[1] = sim_model_->WorldPose().Rot().X();
    base_orientation_[2] = sim_model_->WorldPose().Rot().Y();
    base_orientation_[3] = sim_model_->WorldPose().Rot().Z();

    // IMU data:
    // In this case we are using the IMU sensor which has angular velocities and accelerations defined wrt the trunk/base
    if(this->imu_sensor_ != nullptr)
    {
      imu_orientation_[0] = imu_sensor_->Orientation().W();
      imu_orientation_[1] = imu_sensor_->Orientation().X();
      imu_orientation_[2] = imu_sensor_->Orientation().Y();
      imu_orientation_[3] = imu_sensor_->Orientation().Z();

      imu_ang_vel_[0] = imu_sensor_->AngularVelocity().X();
      imu_ang_vel_[1] = imu_sensor_->AngularVelocity().Y();
      imu_ang_vel_[2] = imu_sensor_->AngularVelocity().Z();

      imu_lin_acc_[0] = imu_sensor_->LinearAcceleration().X();
      imu_lin_acc_[1] = imu_sensor_->LinearAcceleration().Y();
      imu_lin_acc_[2] = imu_sensor_->LinearAcceleration().Z();
    }
    // In this case we need to transform the angular velocities and accelerations from world measurements (Gazebo) to trunk/base (IMU)
    else
    {
      imu_data_.frame_id = gt_data_.frame_id;

      // Calculate base_R_world_
      quaterniond_tmp_.W() = base_orientation_[0];
      quaterniond_tmp_.X() = base_orientation_[1];
      quaterniond_tmp_.Y() = base_orientation_[2];
      quaterniond_tmp_.Z() = base_orientation_[3];
      base_R_world_ = quaterniond_tmp_; // world_R_base
      base_R_world_.Transpose();

      // Orientation
      imu_orientation_[0] = base_orientation_[0];
      imu_orientation_[1] = base_orientation_[1];
      imu_orientation_[2] = base_orientation_[2];
      imu_orientation_[3] = base_orientation_[3];

      // Angular velocities
      vector3d_tmp_.X() = static_cast<double>(base_ang_vel_[0]);
      vector3d_tmp_.Y() = static_cast<double>(base_ang_vel_[1]);
      vector3d_tmp_.Z() = static_cast<double>(base_ang_vel_[2]);
      vector3d_tmp2_ = base_R_world_ * vector3d_tmp_; // base_angular_vel = base_R_world * world_angular_vel
      imu_ang_vel_[0] = vector3d_tmp2_.X();
      imu_ang_vel_[1] = vector3d_tmp2_.Y();
      imu_ang_vel_[2] = vector3d_tmp2_.Z();

      // Linear accelerations
      vector3d_tmp_.X() = static_cast<double>(base_lin_acc_[0]);
      vector3d_tmp_.Y() = static_cast<double>(base_lin_acc_[1]);
      vector3d_tmp_.Z() = static_cast<double>(base_lin_acc_[2]);
      vector3d_tmp2_ = base_R_world_ * vector3d_tmp_; // base_linear_acc = base_R_world * world_linear_acc
      imu_lin_acc_[0] = vector3d_tmp2_.X();
      imu_lin_acc_[1] = vector3d_tmp2_.Y();
      imu_lin_acc_[2] = vector3d_tmp2_.Z();
    }

    // Contact sensors data:
    if(contact_sensors_.size() != 0)
    {
      // Fill the contact sensors reading
      for (unsigned int i = 0; i < contact_sensor_names_.size(); i++)
      {
        const gazebo::msgs::Contacts& contacts = contact_sensors_[contact_sensor_names_[i]]->Contacts();

        if (contacts.contact_size()>=1)
        {
          contact_[i]    = true; // FIXME this seems to be always true!
          force_[i][0]   = contacts.contact(0).wrench(0).body_1_wrench().force().x(); // Local frame force
          force_[i][1]   = contacts.contact(0).wrench(0).body_1_wrench().force().y(); // Local frame force
          force_[i][2]   = contacts.contact(0).wrench(0).body_1_wrench().force().z(); // Local frame force
          normal_[i][0]  = contacts.contact(0).normal(0).x();
          normal_[i][1]  = contacts.contact(0).normal(0).y();
          normal_[i][2]  = contacts.contact(0).normal(0).z();
        }
        else
        {
          contact_[i]    = false;
          force_[i][0]   = 0.0;
          force_[i][1]   = 0.0;
          force_[i][2]   = 0.0;
          normal_[i][0]  = 0.0;
          normal_[i][1]  = 0.0;
          normal_[i][2]  = 0.0;
        }
      }
    }

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
