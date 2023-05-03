//
// Created by kook on 4/21/23.
//

#pragma once

#include "rm_shooter_controllers/shooter_base.h"

namespace rm_shooter_controllers
{
class PushRodController: public Controller<hardware_interface::EffortJointInterface,rm_control::RobotStateInterface>
{
public:
  PushRodController()=default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;

private:
  void stop(const ros::Time& time, const ros::Duration& period) override;
  void push(const ros::Time& time, const ros::Duration& period) override;
  void reachSpeed(double qd_des) override;
  void normalize() override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;

  effort_controllers::JointVelocityController ctrl_friction_l_;
  effort_controllers::JointVelocityController ctrl_friction_r_;
  effort_controllers::JointPositionController ctrl_putter_;

  ros::Time last_shoot_time_;

  double putter_pos_threshold_{}, putter_initial_pos_{}, forward_distance_{};
  bool putter_is_ready_ = true, finish_shoot_ = false;
};
}// namespace rm_shooter_controllers