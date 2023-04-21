#include "rm_shooter_controllers/pneumatic.h"
#include "pluginlib/class_list_macros.hpp"

namespace rm_shooter_controllers
{
bool PneumaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  ros::NodeHandle nh_putter = ros::NodeHandle(controller_nh, "putter");
  ros::NodeHandle nh_pump = ros::NodeHandle(controller_nh, "pump");
  gpio_pub_ = controller_nh.advertise<rm_msgs::GpioData>("/controllers/gpio_controller/command", 1);
  if (!controller_nh.getParam("putter_pos_threshold", putter_pos_threshold_))
    ROS_ERROR("Putter position threshold no defined (namespace: %s)", controller_nh.getNamespace().c_str());
  if (!controller_nh.getParam("forward_distance", forward_distance_))
    ROS_ERROR("Forward distance no defined (namespace: %s)", controller_nh.getNamespace().c_str());
  if (!controller_nh.getParam("pump_duration", pump_duration_))
    ROS_ERROR("Putter duration no defined (namespace: %s)", controller_nh.getNamespace().c_str());
  return (ctrl_trigger_.init(effort_joint_interface_, nh_trigger) &&
          ctrl_putter_.init(effort_joint_interface_, nh_putter) && ctrl_pump_.init(effort_joint_interface_, nh_pump));
}

void PneumaticController::starting(const ros::Time& time)
{
  Controller::starting(time);
  putter_initial_pos_ = ctrl_putter_.joint_.getPosition();
}

void PneumaticController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  if ((((ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) ||
       ((cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST) && ((ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz))))
  {
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
            config_.forward_push_threshold &&
        !start_shoot_)
    {
      ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                               2. * M_PI / static_cast<double>(push_per_rotation_));
      ctrl_trigger_.update(time, period);
      start_shoot_ = true;
    }

    // push bullet to pressure chamber
    if (std::fmod(std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.getPosition()), 2. * M_PI) <
        config_.forward_push_threshold)
    {
      ctrl_putter_.setCommand(putter_initial_pos_ + forward_distance_);
      ctrl_putter_.update(time, period);
      start_pump_ = true;
    }

    // start pumping
    if (std::fmod(std::abs(ctrl_putter_.command_struct_.position_ - ctrl_putter_.getPosition()), 2. * M_PI) <
            putter_pos_threshold_ &&
        start_pump_)
    {
      ctrl_pump_.setCommand(qd_des_);
      ctrl_pump_.update(time, period);
      if (!is_pumping_)
      {
        last_pump_time_ = ros::Time::now();
        is_pumping_ = true;
      }
    }

    // finish shooting
    if ((ros::Time::now() - last_pump_time_).toSec() > pump_duration_ && is_pumping_)
    {
      data_.gpio_state[0] = 1;
      gpio_pub_.publish(data_);
      ctrl_pump_.setCommand(0);
      ctrl_putter_.setCommand(putter_initial_pos_);
      last_shoot_time_ = time;
      is_pumping_ = false;
      start_shoot_ = false;
      start_pump_ = false;
    }
  }
  checkBlock(time);
}

void PneumaticController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
    ctrl_putter_.setCommand(0.);
    ctrl_pump_.setCommand(0.);
  }
}

void PneumaticController::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
  ctrl_putter_.joint_.setCommand(0);
  ctrl_pump_.setCommand(0);
}

void PneumaticController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_trigger_.update(time, period);
  ctrl_putter_.update(time, period);
  ctrl_pump_.update(time, period);
}

}  // namespace rm_shooter_controllers

PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::PneumaticController, controller_interface::ControllerBase)