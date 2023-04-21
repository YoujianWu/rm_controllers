//
// created by guanlin on 2022/9/6
//

#pragma once

#include "rm_shooter_controllers/shooter_base.h"

namespace rm_shooter_controllers
{
class FrictionWheelController
  : public Controller<hardware_interface::EffortJointInterface, rm_control::RobotStateInterface>
{
public:
  FrictionWheelController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void stop(const ros::Time& time, const ros::Duration& period) override;
  void push(const ros::Time& time, const ros::Duration& period) override;
  void reachSpeed(double qd_des) override;
  void normalize() override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;

  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
};
}  // namespace rm_shooter_controllers