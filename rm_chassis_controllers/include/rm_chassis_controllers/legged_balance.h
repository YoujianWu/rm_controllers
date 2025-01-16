//
// Created by kook on 1/15/25.
//

#pragma once

#include <eigen3/Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "rm_chassis_controllers/chassis_base.h"
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
  void updateEstimation(const ros::Time& time);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double vmc_bias_angle_, left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
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

  // sub
  ::ros::Subscriber leg_cmd_subscriber_;
  rm_msgs::LegCmd leg_cmd_;

  double coeff[40][6] = {
    { -1.3907, -4.8818, 4.7517, 10.273, -6.7264, -3.402 },
    { -19.023, -17.566, 71.038, 80.164, -101.8, -56.366 },
    { -46.293, -198.96, -48.799, -19.761, 43.384, 102.98 },
    { -9.0385, -9.5023, -7.0859, -27.734, 0.58893, 15.852 },
    { -59.394, -48.601, 11.631, 54.067, 15.204, -26.803 },
    { -2.1926, -13.319, 4.2646, 0.55447, 3.1883, -5.7891 },
    { -0.58397, 19.074, -63.884, -76.941, 13.759, 80.46 },
    { -0.40989, 0.68562, -5.9955, 0.93794, -5.9916, 6.4847 },
    { -85.575, 349.11, 17.109, -493.75, 134.05, -24.767 },
    { -6.0475, 10.913, 2.2513, -8.0129, 0.45409, -1.0606 },
    { -1.3907, 4.7517, -4.8818, -3.402, -6.7264, 10.273 },
    { -19.023, 71.038, -17.566, -56.366, -101.8, 80.164 },
    { 46.293, 48.799, 198.96, -102.98, -43.384, 19.761 },
    { 9.0385, 7.0859, 9.5023, -15.852, -0.58893, 27.734 },
    { -0.58397, -63.884, 19.074, 80.46, 13.759, -76.941 },
    { -0.40989, -5.9955, 0.68562, 6.4847, -5.9916, 0.93794 },
    { -59.394, 11.631, -48.601, -26.803, 15.204, 54.067 },
    { -2.1926, 4.2646, -13.319, -5.7891, 3.1883, 0.55447 },
    { -85.575, 17.109, 349.11, -24.767, 134.05, -493.75 },
    { -6.0475, 2.2513, 10.913, -1.0606, 0.45409, -8.0129 },
    { 0.0090435, 1.2781, -1.253, -0.89615, 0.14231, 0.20642 },
    { 0.13879, 12.795, -13.418, -13.548, 2.1834, 9.307 },
    { -60.857, 33.016, 4.7912, 13.415, -1.1934, 39.465 },
    { -11.165, 2.6117, -0.31435, 3.642, 1.0136, 4.7979 },
    { 9.3398, 19.015, 5.7716, 42.015, 14.462, -15.105 },
    { 0.34883, 0.9359, -0.080525, 9.3737, -0.9877, -0.22449 },
    { 1.0809, -5.7206, -62.172, 18.535, -21.843, 7.9823 },
    { 0.017778, 0.036894, -3.407, 0.27396, 0.18977, -7.7281 },
    { -81.045, -56.977, 21.766, 60.188, -3.3403, 2.375 },
    { -5.3198, -2.2745, 1.2855, 0.6102, 0.18595, 0.57974 },
    { 0.0090435, -1.253, 1.2781, 0.20642, 0.14231, -0.89615 },
    { 0.13879, -13.418, 12.795, 9.307, 2.1834, -13.548 },
    { 60.857, -4.7912, -33.016, -39.465, 1.1934, -13.415 },
    { 11.165, 0.31435, -2.6117, -4.7979, -1.0136, -3.642 },
    { 1.0809, -62.172, -5.7206, 7.9823, -21.843, 18.535 },
    { 0.017778, -3.407, 0.036894, -7.7281, 0.18977, 0.27396 },
    { 9.3398, 5.7716, 19.015, -15.105, 14.462, 42.015 },
    { 0.34883, -0.080525, 0.9359, -0.22449, -0.9877, 9.3737 },
    { -81.045, 21.766, -56.977, 2.375, -3.3403, 60.188 },
    { -5.3198, 1.2855, -2.2745, 0.57974, 0.18595, 0.6102 },
  };

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> getK(double l_ll, double l_lr)
  {
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> M_;
    for (int i = 0; i < 4; i++)
    {
      int index = i * 10;
      M_(i, 0) = coeff[index + 0][0] + coeff[index + 0][1] * l_ll + coeff[index + 0][2] * l_lr +
                 coeff[index + 0][3] * l_ll * l_ll + coeff[index + 0][4] * l_ll * l_lr +
                 coeff[index + 0][5] * l_lr * l_lr;
      M_(i, 1) = coeff[index + 1][0] + coeff[index + 1][1] * l_ll + coeff[index + 1][2] * l_lr +
                 coeff[index + 1][3] * l_ll * l_ll + coeff[index + 1][4] * l_ll * l_lr +
                 coeff[index + 1][5] * l_lr * l_lr;
      M_(i, 2) = coeff[index + 2][0] + coeff[index + 2][1] * l_ll + coeff[index + 2][2] * l_lr +
                 coeff[index + 2][3] * l_ll * l_ll + coeff[index + 2][4] * l_ll * l_lr +
                 coeff[index + 2][5] * l_lr * l_lr;
      M_(i, 3) = coeff[index + 3][0] + coeff[index + 31][1] * l_ll + coeff[index + 3][2] * l_lr +
                 coeff[index + 3][3] * l_ll * l_ll + coeff[index + 3][4] * l_ll * l_lr +
                 coeff[index + 3][5] * l_lr * l_lr;
      M_(i, 4) = coeff[index + 4][0] + coeff[index + 4][1] * l_ll + coeff[index + 4][2] * l_lr +
                 coeff[index + 4][3] * l_ll * l_ll + coeff[index + 4][4] * l_ll * l_lr +
                 coeff[index + 4][5] * l_lr * l_lr;
      M_(i, 5) = coeff[index + 5][0] + coeff[index + 5][1] * l_ll + coeff[index + 5][2] * l_lr +
                 coeff[index + 5][3] * l_ll * l_ll + coeff[index + 5][4] * l_ll * l_lr +
                 coeff[index + 5][5] * l_lr * l_lr;
      M_(i, 6) = coeff[index + 6][0] + coeff[index + 6][1] * l_ll + coeff[index + 6][2] * l_lr +
                 coeff[index + 6][3] * l_ll * l_ll + coeff[index + 6][4] * l_ll * l_lr +
                 coeff[index + 6][5] * l_lr * l_lr;
      M_(i, 7) = coeff[index + 7][0] + coeff[index + 7][1] * l_ll + coeff[index + 7][2] * l_lr +
                 coeff[index + 7][3] * l_ll * l_ll + coeff[index + 7][4] * l_ll * l_lr +
                 coeff[index + 7][5] * l_lr * l_lr;
      M_(i, 8) = coeff[index + 8][0] + coeff[index + 8][1] * l_ll + coeff[index + 8][2] * l_lr +
                 coeff[index + 8][3] * l_ll * l_ll + coeff[index + 8][4] * l_ll * l_lr +
                 coeff[index + 8][5] * l_lr * l_lr;
      M_(i, 9) = coeff[index + 9][0] + coeff[index + 9][1] * l_ll + coeff[index + 9][2] * l_lr +
                 coeff[index + 9][3] * l_ll * l_ll + coeff[index + 9][4] * l_ll * l_lr +
                 coeff[index + 9][5] * l_lr * l_lr;
    }
    return M_;
  }

  void legCmdCallback(const rm_msgs::LegCmdConstPtr& msg);
};

}  // namespace rm_chassis_controllers
