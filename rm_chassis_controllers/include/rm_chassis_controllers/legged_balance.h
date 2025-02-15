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
    { -1.351, -3.9226, 3.5188, 6.757, -0.61748, -5.507 },
    { -11.227, -6.7523, 34.664, 18.841, -8.3578, -52.395 },
    { -13.241, 30.056, -4.1229, -60.202, -11.364, 13.395 },
    { -4.1474, 20.18, -0.75175, -29.233, -4.4657, 2.5627 },
    { -16.966, -87.564, 6.8476, -14.335, 8.8411, -12.412 },
    { -6.1565, -4.5825, 2.4804, -7.8908, 3.9234, -5.295 },
    { -2.7617, 23.594, -22.606, -52.172, 99.589, 3.1953 },
    { -0.096557, 0.44223, -6.2153, -2.7465, 7.8478, 3.7999 },
    { -84.616, 296.04, 0.51163, -281.36, 36.527, 6.5022 },
    { -5.7427, 10.949, 1.172, -5.5661, -0.75554, -0.84077 },
    { -1.351, 3.5188, -3.9226, -5.507, -0.61748, 6.757 },
    { -11.227, 34.664, -6.7523, -52.395, -8.3578, 18.841 },
    { 13.241, 4.1229, -30.056, -13.395, 11.364, 60.202 },
    { 4.1474, 0.75175, -20.18, -2.5627, 4.4657, 29.233 },
    { -2.7617, -22.606, 23.594, 3.1953, 99.589, -52.172 },
    { -0.096557, -6.2153, 0.44223, 3.7999, 7.8478, -2.7465 },
    { -16.966, 6.8476, -87.564, -12.412, 8.8411, -14.335 },
    { -6.1565, 2.4804, -4.5825, -5.295, 3.9234, -7.8908 },
    { -84.616, 0.51163, 296.04, 6.5022, 36.527, -281.36 },
    { -5.7427, 1.172, 10.949, -0.84077, -0.75554, -5.5661 },
    { 0.14938, -0.039499, -0.42653, -0.23658, 0.22451, 0.10072 },
    { 1.1198, -1.3027, -3.8767, -0.053748, 3.0152, 2.8102 },
    { -13.114, -2.7885, -0.63752, 6.1063, 1.479, 0.0907 },
    { -4.566, -0.94741, 0.20955, 2.376, 0.10553, -1.1 },
    { 3.3222, 25.807, -1.0057, 0.33183, 17.546, 1.0722 },
    { 1.0677, -1.5679, -0.1714, 5.8985, 1.4698, 0.0075506 },
    { 0.07617, -2.4549, -28.819, 6.9264, -17.922, -6.6397 },
    { 0.01054, -0.13883, -1.8486, 0.62762, -2.3533, -3.9984 },
    { -80.424, -39.044, 4.0529, 42.905, -0.033161, 10.558 },
    { -5.2914, -1.8144, 0.48476, 1.1403, 0.24515, 0.58496 },
    { 0.14938, -0.42653, -0.039499, 0.10072, 0.22451, -0.23658 },
    { 1.1198, -3.8767, -1.3027, 2.8102, 3.0152, -0.053748 },
    { 13.114, 0.63752, 2.7885, -0.0907, -1.479, -6.1063 },
    { 4.566, -0.20955, 0.94741, 1.1, -0.10553, -2.376 },
    { 0.07617, -28.819, -2.4549, -6.6397, -17.922, 6.9264 },
    { 0.01054, -1.8486, -0.13883, -3.9984, -2.3533, 0.62762 },
    { 3.3222, -1.0057, 25.807, 1.0722, 17.546, 0.33183 },
    { 1.0677, -0.1714, -1.5679, 0.0075506, 1.4698, 5.8985 },
    { -80.424, 4.0529, -39.044, 10.558, -0.033161, 42.905 },
    { -5.2914, 0.48476, -1.8144, 0.58496, 0.24515, 1.1403 },
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
