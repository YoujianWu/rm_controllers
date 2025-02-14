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
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  double wheel_radius_ = 0.09, wheel_base_;
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
  double roll_, pitch_, yaw_;

  control_toolbox::Pid pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_length_diff_, pid_roll_, pid_center_gravity_;

  // Slippage_detection
  Eigen::Matrix<double, 2, 2> A_, B_, H_, Q_, R_;
  Eigen::Matrix<double, 2, 1> X_, U_;
  int i = 0, sample_times_ = 3;
  std::shared_ptr<KalmanFilter<double>> kalmanFilterPtr_;

  // sub
  ::ros::Subscriber leg_cmd_subscriber_;
  rm_msgs::LegCmd leg_cmd_;
  void legCmdCallback(const rm_msgs::LegCmdConstPtr& msg);

  double coeff[40][6] = {
    { -1.3538, -2.9948, 2.6167, 6.697, -1.7596, -4.3583 },
    { -11.237, -0.97328, 28.977, 16.145, -14.232, -44.041 },
    { -51.59, -202.2, -40.016, -15.674, 28.922, 119.86 },
    { -9.606, -8.5704, -6.7153, -29.307, -1.1828, 18.192 },
    { -16.762, -89.849, 9.0144, 40.43, 27.386, -19.141 },
    { -6.1665, -3.8358, 2.7522, -0.54842, 3.1754, -5.8833 },
    { -3.0839, 30.519, -28.541, -67.633, 53.537, -3.2078 },
    { -0.10456, 0.95551, -7.6039, -4.0674, 3.2943, 3.4215 },
    { -84.412, 287.07, 7.7923, -343.41, 100.63, 7.5646 },
    { -5.7326, 10.505, 1.5386, -8.7018, 2.057, -0.38978 },
    { -1.3538, 2.6167, -2.9948, -4.3583, -1.7596, 6.697 },
    { -11.237, 28.977, -0.97328, -44.041, -14.232, 16.145 },
    { 51.59, 40.016, 202.2, -119.86, -28.922, 15.674 },
    { 9.606, 6.7153, 8.5704, -18.192, 1.1828, 29.307 },
    { -3.0839, -28.541, 30.519, -3.2078, 53.537, -67.633 },
    { -0.10456, -7.6039, 0.95551, 3.4215, 3.2943, -4.0674 },
    { -16.762, 9.0144, -89.849, -19.141, 27.386, 40.43 },
    { -6.1665, 2.7522, -3.8358, -5.8833, 3.1754, -0.54842 },
    { -84.412, 7.7923, 287.07, 7.5646, 100.63, -343.41 },
    { -5.7326, 1.5386, 10.505, -0.38978, 2.057, -8.7018 },
    { 0.14923, 0.83485, -1.3003, -0.99084, 0.20816, 0.8716 },
    { 1.119, 4.0506, -9.2262, -5.9314, 2.93, 8.7728 },
    { -60.164, 36.267, 5.2201, 10.71, -5.0949, 40.316 },
    { -11.089, 2.649, -0.23394, 3.8957, 0.80463, 4.7481 },
    { 2.893, 34.375, 0.68144, 11.18, 23.794, -0.96119 },
    { 1.0537, -0.30504, -0.14086, 7.748, 0.77895, -0.084564 },
    { 0.50495, -3.9187, -37.624, 8.8104, -24.344, -17.107 },
    { 0.023883, -0.086252, -3.1922, 0.64261, -1.6951, -5.7343 },
    { -80.426, -51.474, 16.531, 45.945, -2.8175, 10.137 },
    { -5.2914, -2.5092, 1.1807, 0.95395, 0.23272, 0.77937 },
    { 0.14923, -1.3003, 0.83485, 0.8716, 0.20816, -0.99084 },
    { 1.119, -9.2262, 4.0506, 8.7728, 2.93, -5.9314 },
    { 60.164, -5.2201, -36.267, -40.316, 5.0949, -10.71 },
    { 11.089, 0.23394, -2.649, -4.7481, -0.80463, -3.8957 },
    { 0.50495, -37.624, -3.9187, -17.107, -24.344, 8.8104 },
    { 0.023883, -3.1922, -0.086252, -5.7343, -1.6951, 0.64261 },
    { 2.893, 0.68144, 34.375, -0.96119, 23.794, 11.18 },
    { 1.0537, -0.14086, -0.30504, -0.084564, 0.77895, 7.748 },
    { -80.426, 16.531, -51.474, 10.137, -2.8175, 45.945 },
    { -5.2914, 1.1807, -2.5092, 0.77937, 0.23272, 0.95395 },
  };

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> getK(double l_ll, double l_lr)
  {
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> m;
    for (int i = 0; i < 4; i++)
    {
      int index = i * 10;
      m(i, 0) = coeff[index + 0][0] + coeff[index + 0][1] * l_ll + coeff[index + 0][2] * l_lr +
                coeff[index + 0][3] * l_ll * l_ll + coeff[index + 0][4] * l_ll * l_lr +
                coeff[index + 0][5] * l_lr * l_lr;
      m(i, 1) = coeff[index + 1][0] + coeff[index + 1][1] * l_ll + coeff[index + 1][2] * l_lr +
                coeff[index + 1][3] * l_ll * l_ll + coeff[index + 1][4] * l_ll * l_lr +
                coeff[index + 1][5] * l_lr * l_lr;
      m(i, 2) = coeff[index + 2][0] + coeff[index + 2][1] * l_ll + coeff[index + 2][2] * l_lr +
                coeff[index + 2][3] * l_ll * l_ll + coeff[index + 2][4] * l_ll * l_lr +
                coeff[index + 2][5] * l_lr * l_lr;
      m(i, 3) = coeff[index + 3][0] + coeff[index + 3][1] * l_ll + coeff[index + 3][2] * l_lr +
                coeff[index + 3][3] * l_ll * l_ll + coeff[index + 3][4] * l_ll * l_lr +
                coeff[index + 3][5] * l_lr * l_lr;
      m(i, 4) = coeff[index + 4][0] + coeff[index + 4][1] * l_ll + coeff[index + 4][2] * l_lr +
                coeff[index + 4][3] * l_ll * l_ll + coeff[index + 4][4] * l_ll * l_lr +
                coeff[index + 4][5] * l_lr * l_lr;
      m(i, 5) = coeff[index + 5][0] + coeff[index + 5][1] * l_ll + coeff[index + 5][2] * l_lr +
                coeff[index + 5][3] * l_ll * l_ll + coeff[index + 5][4] * l_ll * l_lr +
                coeff[index + 5][5] * l_lr * l_lr;
      m(i, 6) = coeff[index + 6][0] + coeff[index + 6][1] * l_ll + coeff[index + 6][2] * l_lr +
                coeff[index + 6][3] * l_ll * l_ll + coeff[index + 6][4] * l_ll * l_lr +
                coeff[index + 6][5] * l_lr * l_lr;
      m(i, 7) = coeff[index + 7][0] + coeff[index + 7][1] * l_ll + coeff[index + 7][2] * l_lr +
                coeff[index + 7][3] * l_ll * l_ll + coeff[index + 7][4] * l_ll * l_lr +
                coeff[index + 7][5] * l_lr * l_lr;
      m(i, 8) = coeff[index + 8][0] + coeff[index + 8][1] * l_ll + coeff[index + 8][2] * l_lr +
                coeff[index + 8][3] * l_ll * l_ll + coeff[index + 8][4] * l_ll * l_lr +
                coeff[index + 8][5] * l_lr * l_lr;
      m(i, 9) = coeff[index + 9][0] + coeff[index + 9][1] * l_ll + coeff[index + 9][2] * l_lr +
                coeff[index + 9][3] * l_ll * l_ll + coeff[index + 9][4] * l_ll * l_lr +
                coeff[index + 9][5] * l_lr * l_lr;
    }
    return m;
  }
};
}  // namespace rm_chassis_controllers
