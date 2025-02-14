//
// Created by kook on 1/15/25.
//

#include "rm_chassis_controllers/legged_balance.h"
#include "rm_chassis_controllers/vmc/leg_conv.h"
#include "rm_chassis_controllers/vmc/leg_pos.h"
#include "rm_chassis_controllers/vmc/leg_spd.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/BalanceState.h>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool LeggedBalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  std::string left_wheel_joint, right_wheel_joint, left_front_leg_joint, left_back_leg_joint, right_front_leg_joint,
      right_back_leg_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("left/front_leg_joint", left_front_leg_joint) ||
      !controller_nh.getParam("left/back_leg_joint", left_back_leg_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("right/front_leg_joint", right_front_leg_joint) ||
      !controller_nh.getParam("right/back_leg_joint", right_back_leg_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  left_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_front_leg_joint);
  left_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_back_leg_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  right_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_front_leg_joint);
  right_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_back_leg_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);
  if (!controller_nh.getParam("vmc_bias_angle", vmc_bias_angle_))
  {
    ROS_ERROR("Params vmc_bias_angle doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (controller_nh.hasParam("pid_left_leg"))
  {
    if (!pid_left_leg_.init(ros::NodeHandle(controller_nh, "pid_left_leg")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_right_leg"))
  {
    if (!pid_right_leg_.init(ros::NodeHandle(controller_nh, "pid_right_leg")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_theta_diff"))
  {
    if (!pid_theta_diff_.init(ros::NodeHandle(controller_nh, "pid_theta_diff")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_roll"))
  {
    if (!pid_roll_.init(ros::NodeHandle(controller_nh, "pid_roll")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_center_gravity"))
  {
    if (!pid_center_gravity_.init(ros::NodeHandle(controller_nh, "pid_center_gravity")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_length_diff"))
  {
    if (!pid_length_diff_.init(ros::NodeHandle(controller_nh, "pid_length_diff")))
    {
      return false;
    }
  }

  // Slippage detection
  A_ << 1, 0, 0, 1;
  H_ << 1, 0, 0, 1;
  Q_ << 0.5, 0, 0, 0.5;
  R_ << 100, 0, 0, 100;
  B_.setZero();
  X_.setZero();
  U_.setZero();
  kalmanFilterPtr_ = std::make_shared<KalmanFilter<double>>(A_, B_, H_, Q_, R_);
  kalmanFilterPtr_->clear(X_);

  // sub
  leg_cmd_subscriber_ =
      root_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 1, &LeggedBalanceController::legCmdCallback, this);
  balance_mode_ = BalanceMode::NORMAL;
  return true;
}

void LeggedBalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  updateEstimation(time, period);
  switch (balance_mode_)
  {
    case BalanceMode::NORMAL:
    {
      normal(time, period);
      break;
    }
    case BalanceMode::BLOCK:
    {
      block(time, period);
      break;
    }
  }
}

void LeggedBalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
  if (balance_state_changed_)
  {
    ROS_INFO("[balance] Enter NOMAl");
    balance_state_changed_ = false;
  }

  // PID
  double T_theta_diff = pid_theta_diff_.computeCommand(x_[4] - x_[6], period);
  double F_length_diff = pid_length_diff_.computeCommand(left_pos_[0] - right_pos_[0], period);
  double leg_aver = (left_pos_[0] + right_pos_[0]) / 2;

  // set target state
  yaw_des_ += vel_cmd_.z * period.toSec();
  position_des_ += vel_cmd_.x * period.toSec();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  auto x = x_;
  x(0) -= position_des_;
  x(2) = angles::shortest_angular_distance(yaw_des_, x_(2));
  if (state_ != RAW)
    x(1) -= vel_cmd_.x;
  x(3) -= vel_cmd_.z;
  k_ = getK(left_pos_[0], right_pos_[0]);
  u = k_ * (-x);
  left_wheel_joint_handle_.setCommand(u(0));
  right_wheel_joint_handle_.setCommand(u(1));

  // Leg control
  Eigen::Matrix<double, 2, 3> j;
  Eigen::Matrix<double, 3, 1> p;
  Eigen::Matrix<double, 2, 1> F_leg, F_bl;
  double F_roll, F_gravity, F_inertial;
  F_leg[0] = pid_left_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) - F_length_diff;
  F_leg[1] = pid_right_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) + F_length_diff;
  F_roll = pid_roll_.computeCommand(0 - roll_, period);
  F_inertial = (0.5 * 11.7) * leg_aver * 0.5 * 0.49 * x[1] * x[3];
  F_gravity = 0.5 * 11.7 * 9.8;
  // clang-format off
  j << 1, cos(x_(4)), -1,
      -1, cos(x_(6)), 1;
  // clang-format on
  p << F_roll, F_gravity, F_inertial;
  F_bl = j * p + F_leg;

  double left_T[2], right_T[2];
  leg_conv(F_bl[0], u(2) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_bl[1], u(3) + T_theta_diff, right_angle[0], right_angle[1], right_T);
  left_front_leg_joint_handle_.setCommand(left_T[1]);
  right_front_leg_joint_handle_.setCommand(right_T[1]);
  left_back_leg_joint_handle_.setCommand(left_T[0]);
  right_back_leg_joint_handle_.setCommand(right_T[0]);
}

void LeggedBalanceController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base_,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    left_wheel_joint_handle_.setCommand(0.);
    right_wheel_joint_handle_.setCommand(0.);
    left_front_leg_joint_handle_.setCommand(0.);
    left_back_leg_joint_handle_.setCommand(0.);
    right_front_leg_joint_handle_.setCommand(0.);
    right_back_leg_joint_handle_.setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imu_quaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                               imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imu_quaternion);
  odom2base = odom2imu * imu2base;
  quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

  // vmc
  left_angle[0] =
      vmc_bias_angle_ + left_back_leg_joint_handle_.getPosition();  // [0]:back_vmc_joint [1]:front_vmc_joint
  left_angle[1] = left_front_leg_joint_handle_.getPosition() + M_PI - vmc_bias_angle_;
  right_angle[0] = vmc_bias_angle_ + right_back_leg_joint_handle_.getPosition();
  right_angle[1] = right_front_leg_joint_handle_.getPosition() + M_PI - vmc_bias_angle_;
  leg_pos(left_angle[0], left_angle[1], left_pos_);
  leg_pos(right_angle[0], right_angle[1], right_pos_);
  leg_spd(left_back_leg_joint_handle_.getVelocity(), left_front_leg_joint_handle_.getVelocity(), left_angle[0],
          left_angle[1], left_spd_);
  leg_spd(right_back_leg_joint_handle_.getVelocity(), right_front_leg_joint_handle_.getVelocity(), right_angle[0],
          right_angle[1], right_spd_);

  // Slippage_detection
  double leftWheelVel = (left_wheel_joint_handle_.getVelocity() - gyro.z + left_spd_[0]) * wheel_radius_;
  double rightWheelVel = (right_wheel_joint_handle_.getVelocity() + gyro.z + right_spd_[0]) * wheel_radius_;
  double leftWheelVelAbsolute =
      leftWheelVel + left_pos_[0] * left_spd_[1] * cos(left_pos_[1]) + left_spd_[0] * sin(left_pos_[1]);
  double rightWheelVelAbsolute =
      rightWheelVel + right_pos_[0] * right_spd_[1] * cos(right_pos_[1]) + right_spd_[0] * sin(right_pos_[1]);

  double wheel_vel_aver = (leftWheelVelAbsolute + rightWheelVelAbsolute) / 2.;
  if (i >= sample_times_)
  {  // oversampling
    i = 0;
    X_(0) = wheel_vel_aver;
    X_(1) = imu_handle_.getLinearAccelerationCovariance()[0];
    kalmanFilterPtr_->predict(U_);
    kalmanFilterPtr_->update(X_);
  }
  else
  {
    kalmanFilterPtr_->predict(U_);
    i++;
  }
  auto x_hat = kalmanFilterPtr_->getState();

  // update state
  x_[1] = x_hat[0];
  x_[0] += x_[1] * period.toSec();
  x_[2] = yaw_;
  x_[3] = angular_vel_base_.z;
  x_[4] = left_pos_[1] + pitch_;
  x_[5] = left_spd_[1] + angular_vel_base_.y;
  x_[6] = right_pos_[1] + pitch_;
  x_[7] = right_spd_[1] + angular_vel_base_.y;
  x_[8] = pitch_;
  x_[9] = angular_vel_base_.y;
}

geometry_msgs::Twist LeggedBalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[1];
  twist.angular.z = x_[3];
  return twist;
}

void LeggedBalanceController::legCmdCallback(const rm_msgs::LegCmdConstPtr& msg)
{
  leg_cmd_ = *msg;
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::LeggedBalanceController, controller_interface::ControllerBase)
