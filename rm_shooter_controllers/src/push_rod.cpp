//
// Created by kook on 4/21/23.
//

#include "rm_shooter_controllers/push_rod.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm_shooter_controllers
{
bool PushRodController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  ros::NodeHandle nh_putter = ros::NodeHandle(controller_nh, "putter");
  if (!controller_nh.getParam("putter_pos_threshold", putter_pos_threshold_))
    ROS_ERROR("Putter position threshold no defined (namespace: %s)", controller_nh.getNamespace().c_str());
  if (!controller_nh.getParam("forward_distance", forward_distance_))
    ROS_ERROR("Forward distance no defined (namespace: %s)", controller_nh.getNamespace().c_str());
  return (ctrl_friction_l_.init(effort_joint_interface_,nh_friction_l) &&
          ctrl_friction_r_.init(effort_joint_interface_,nh_friction_r) &&
          ctrl_trigger_.init(effort_joint_interface_, nh_trigger) &&
          ctrl_putter_.init(effort_joint_interface_, nh_putter));
}

void PushRodController::starting(const ros::Time& time)
{
  Controller::starting(time);
  putter_initial_pos_ = ctrl_putter_.joint_.getPosition();
}

void PushRodController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  if ((cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST ||
       (ctrl_friction_l_.joint_.getVelocity() >= push_qd_threshold_ * ctrl_friction_l_.command_ &&
        ctrl_friction_l_.joint_.getVelocity() > M_PI &&
        ctrl_friction_r_.joint_.getVelocity() <= push_qd_threshold_ * ctrl_friction_r_.command_ &&
        ctrl_friction_r_.joint_.getVelocity() < -M_PI)) &&
      (time - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  {
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
            config_.forward_push_threshold)
    {
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                               2. * M_PI / static_cast<double>(push_per_rotation_));
      ctrl_trigger_.update(time, period);
      start_shoot_ = true;
    }

    // push bullet towards friction wheels
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
        config_.forward_push_threshold)
    {
      ctrl_putter_.setCommand(putter_initial_pos_ + forward_distance_);
      ctrl_putter_.update(time, period);
      finish_shoot_ = true;
    }

    // finish shooting
    if(finish_shoot_)
    {
      ctrl_putter_.setCommand(putter_initial_pos_);
      last_shoot_time_ = time;
      finish_shoot_ = false;
    }
  }

}

void PushRodController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
    ctrl_putter_.setCommand(putter_initial_pos_);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
  }
}

void PushRodController::reachSpeed(double qd_des)
{
  ctrl_friction_l_.setCommand(qd_des + config_.lf_extra_rotat_speed);
  ctrl_friction_r_.setCommand(-qd_des);
}

void PushRodController::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
  ctrl_putter_.joint_.setCommand(0);
}

void PushRodController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
  ctrl_putter_.update(time, period);
}

} // namespace rm_shooter_controllers
