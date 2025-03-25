//
// Created by kook on 1/15/25.
//

#pragma once

#include <eigen3/Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_common/filters/kalman_filter.h"
#include "rm_msgs/LegCmd.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class LeggedBalanceController
  : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                       hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    BLOCK
  };

public:
  LeggedBalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 6;
  static const int CONTROL_DIM = 2;

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  double wheel_radius_ = 0.09, wheel_track_ = 0.49;
  double body_mass_ = 11.7, g_ = 9.81;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  int balance_mode_;
  ros::Time block_time_, last_block_time_;
  double block_angle_, block_duration_, block_velocity_, block_effort_, anti_block_effort_, block_overtime_;
  bool balance_state_changed_ = false, maybe_block_ = false;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_front_leg_joint_handle_,
      left_back_leg_joint_handle_, right_front_leg_joint_handle_, right_back_leg_joint_handle_;

  geometry_msgs::Vector3 angular_vel_base_;
  double roll_, pitch_, yaw_, yaw_total_;

  control_toolbox::Pid pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_length_diff_, pid_roll_, pid_center_gravity_,
      pid_yaw_pos_, pid_yaw_spd_;

  // Slippage_detection
  Eigen::Matrix<double, 2, 2> A_, B_, H_, Q_, R_;
  Eigen::Matrix<double, 2, 1> X_, U_;
  int i = 0, sample_times_ = 3;
  std::shared_ptr<KalmanFilter<double>> kalmanFilterPtr_;

  // sub
  ::ros::Subscriber leg_cmd_subscriber_;
  rm_msgs::LegCmd leg_cmd_;
  void legCmdCallback(const rm_msgs::LegCmdConstPtr& msg);

  double coeff[12][4] = {
    { -544.6490, 552.9494, -235.4140, -13.3712 },   { 6.5588, -12.2738, 0.8614, -4.7964 },
    { 4.1117, -5.3290, 2.4835, -2.0719 },           { 39.8366, -39.8586, 14.5769, -7.01527 },
    { 1.0e+03 * -3.4554, 3.4114, -1.2339, 0.0166 }, { -71.1485, 71.4427, -27.2336, 1.3491 },
    { 51.4494, -44.4102, 14.7482, 1.5240 },         { -7.4005, 7.4072, -3.4394, 0.0424 },
    { -3.0813, 3.0653, -1.1055, -0.0286 },          { -8.8500, 8.7755, -3.1565, -0.1142 },
    { -24.1423, 55.6464, -33.5148, 53.1938 },       { 1.1982, -0.3320, -0.2435, 1.4912 },
  };

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> getK(double L0)
  {
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k;
    double m3 = L0 * L0 * L0;
    double m2 = L0 * L0;
    double m1 = L0;
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        k(i, j) = coeff[i * 6 + j][0] * m3 + coeff[i * 6 + j][1] * m2 + coeff[i * 6 + j][2] * m1 + coeff[i * 6 + j][3];
      }
    }
    return k;
  }
};
}  // namespace rm_chassis_controllers
