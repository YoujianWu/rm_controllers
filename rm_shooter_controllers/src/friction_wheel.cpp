//
// created by guanlin on 2022/9/6
//

#include "rm_shooter_controllers/friction_wheel.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm_shooter_controllers
{
bool FrictionWheelController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  return (ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) &&
          ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) &&
          ctrl_trigger_.init(effort_joint_interface_, nh_trigger));
}

void FrictionWheelController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
  }
}

void FrictionWheelController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  if ((cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) ||
      (ctrl_friction_l_.joint_.getVelocity() >= push_qd_threshold_ * ctrl_friction_l_.command_ &&
       ctrl_friction_l_.joint_.getVelocity() > M_PI &&
       ctrl_friction_r_.joint_.getVelocity() <= push_qd_threshold_ * ctrl_friction_r_.command_ &&
       ctrl_friction_r_.joint_.getVelocity() < -M_PI && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz))
  {  // Time to shoot!!!
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
        config_.forward_push_threshold)
    {
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                               2. * M_PI / static_cast<double>(push_per_rotation_));
      last_shoot_time_ = time;
    }
  }
  else
    ROS_DEBUG("[Shooter] Wait for friction wheel");
  checkBlock(time);
}

void FrictionWheelController::reachSpeed(double qd_des)
{
  ctrl_friction_l_.setCommand(qd_des + config_.lf_extra_rotat_speed);
  ctrl_friction_r_.setCommand(-qd_des);
}

void FrictionWheelController::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
}

void FrictionWheelController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
}

}  // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::FrictionWheelController, controller_interface::ControllerBase)