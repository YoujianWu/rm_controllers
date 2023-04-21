#pragma once

#include "rm_shooter_controllers/shooter_base.h"
#include "rm_msgs/GpioData.h"

namespace rm_shooter_controllers
{
class PneumaticController : public Controller<hardware_interface::EffortJointInterface, rm_control::RobotStateInterface>
{
public:
  PneumaticController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;

private:
  void stop(const ros::Time& time, const ros::Duration& period) override;
  void push(const ros::Time& time, const ros::Duration& period) override;
  void normalize() override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;

  effort_controllers::JointPositionController ctrl_putter_;
  effort_controllers::JointVelocityController ctrl_pump_;

  ros::Time last_pump_time_;
  ros::Publisher gpio_pub_;

  rm_msgs::GpioData data_;

  double putter_pos_threshold_{}, pump_duration_{}, putter_initial_pos_{}, forward_distance_{};
  bool is_pumping_, start_shoot_, start_pump_ = false;
};
}  // namespace rm_shooter_controllers
