#include <cmath>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/RateThrust.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cascaded_pid_control_node.hpp"

namespace cascaded_pid_control {

  inline geometry_msgs::Vector3 MsgVec3(const Eigen::Vector3d& vec) {
    geometry_msgs::Vector3 result;
    result.x = vec[0];
    result.y = vec[1];
    result.z = vec[2];
    return result;
  }

  inline double Constrain(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    if (value > max) {
      return max;
    }
    return value;
  }

  double CascadedPidControl::AltitudeControl(
    const mav_msgs::EigenOdometry& current,
    const mav_msgs::EigenTrajectoryPoint& cmd,
    const Eigen::Vector3d &accel_ff,
    double dt) {

      Eigen::Matrix3d rot_mat = current.orientation_W_B.toRotationMatrix();

      double dz = cmd.position_W[2] - current.position_W[2];
      double dvz = cmd.velocity_W[2] - current.getVelocityWorld()[2];
      double z_dot_dot = kp_z_ * dz + kd_z_ * dvz + accel_ff[2];

      double result;
      result = z_dot_dot * mass_ / rot_mat(2, 2);

      // clamp value
      result = Constrain(result, min_thrust_, max_thrust_);
      return result;
  }

  Eigen::Vector3d CascadedPidControl::LateralPositionControl(
    const mav_msgs::EigenOdometry& current,
    const mav_msgs::EigenTrajectoryPoint& cmd,
    const Eigen::Vector3d& accel_ff,
    double dt) {

    double err_x = cmd.position_W[0] - current.position_W[0];
    double err_y = cmd.position_W[1] - current.position_W[1];

    double err_vx = cmd.velocity_W[0] - current.getVelocityWorld()[0];
    double err_vy = cmd.velocity_W[1] - current.getVelocityWorld()[1];
    
    Eigen::Vector3d result;
    result << Constrain(kp_x_ * err_x + kd_x_ * err_vx + accel_ff[0], -max_abs_accel_x_, max_abs_accel_x_),
              Constrain(kp_y_ * err_y + kd_y_ * err_vy + accel_ff[1], -max_abs_accel_y_, max_abs_accel_y_),
              0;

    return result;
  }

  Eigen::Vector3d CascadedPidControl::AttitudeControl(
      const mav_msgs::EigenOdometry& pose,
      const Eigen::Vector3d& accel_cmd,
      double thrust_cmd,
      double dt) {
    Eigen::Matrix3d rot_mat = pose.orientation_W_B.toRotationMatrix();

    double bx = rot_mat(0, 2);
    double by = rot_mat(1, 2);
    
    double c = thrust_cmd / mass_;
    double bx_cmd = accel_cmd[0] / c;
    double by_cmd = accel_cmd[1] / c;

    // clamp bx_cmd and by_cmd be with [-sqrt(2)/2,  sqrt(2)/2]
    // this will result in forcing angle between the thrust vector and x/y axis of world frame 
    // be in [45deg, 135deg] (this is not the same with constraining roll and pitch angle)
    bx_cmd = Constrain(bx_cmd, -0.707, 0.707);
    by_cmd = Constrain(by_cmd, -0.707, 0.707);
    
    double bx_dot = kp_pitch_ * (bx_cmd - bx);
    double by_dot = kp_roll_ * (by_cmd - by);

    double k = 1.0 / rot_mat(2, 2);
    Eigen::Vector3d pq_rate_cmd;
    pq_rate_cmd << k * (rot_mat(1, 0) * bx_dot - rot_mat(0, 0) * by_dot),
                   k * (rot_mat(1, 1) * bx_dot - rot_mat(0, 1) * by_dot),
                   0;

    return pq_rate_cmd;
  }

  double CascadedPidControl::YawControl(
    const mav_msgs::EigenOdometry& current,
    double yaw_cmd,
    double dt) {
    double current_yaw = current.getYaw();
    double err_yaw = yaw_cmd - current_yaw;
    while (err_yaw > PI) {
      err_yaw -= 2 * PI;
    }
    while (err_yaw < -PI) {
      err_yaw += 2 * PI;
    }
    return kp_yaw_ * err_yaw;
  }

  void CascadedPidControl::OdometryCallback(const nav_msgs::OdometryConstPtr &ptr) {
    if (controller_active_) {
      mav_msgs::EigenOdometry current;
      mav_msgs::eigenOdometryFromMsg(*ptr, &current);
      double current_time = ptr->header.stamp.toSec();
      double dt = current_time - last_odometry_time_;

      double thrust = AltitudeControl(current, set_point_, accel_ff_, dt);
      Eigen::Vector3d accel_cmd = LateralPositionControl(current, set_point_, accel_ff_, dt);
      Eigen::Vector3d pqr_rate_cmd = AttitudeControl(current, accel_cmd, thrust, dt);
      pqr_rate_cmd[2] = YawControl(current, set_point_.getYaw(), dt);

      // ROS_INFO_STREAM(thrust<<" "<<accel_cmd<<" "<<pqr_rate_cmd);

      // control command is published as mav_msgs/RateThrust
      mav_msgs::RateThrust rate_thrust;
      rate_thrust.header.stamp = ros::Time::now();
      rate_thrust.header.frame_id = "uav/imu";
      rate_thrust.thrust.z = thrust;
      mav_msgs::vectorEigenToMsg(pqr_rate_cmd, &rate_thrust.angular_rates);

      if (publish_debug_topic_) {
        PublishDebugTopic(current);
      }

      rate_thrust_pub_.publish(rate_thrust);
      last_odometry_time_ = current_time;
    }
  }

  void CascadedPidControl::Init() {
    dynamic_reconfigure::Server<CascadedPidConfig>::CallbackType cb;
    cb = boost::bind(&CascadedPidControl::DynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(cb);

    controller_active_ = false;
    last_odometry_time_ = 0;
    accel_ff_ << 0, 0, GRAVITY_CONST;
    publish_debug_topic_ = private_nh_.param("publish_debug_topic", false);

    rate_thrust_pub_ = private_nh_.advertise<mav_msgs::RateThrust>("rateThrust", 1);
    odometry_sub_ = private_nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &CascadedPidControl::OdometryCallback, this);
    trajectory_sub_ = private_nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1, &CascadedPidControl::TrajectoryCallback, this);

    if (publish_debug_topic_) {
      position_setpoint_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("positionSetpoint", 8);
      velocity_setpoint_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("velocitySetpoint", 8);
      attitude_setpoint_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("attitudeSetpoint", 8);
      position_error_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("positionError", 8);
      velocity_error_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("velocityError", 8);
      attitude_error_pub_ = private_nh_.advertise<geometry_msgs::Vector3>("attitudeError", 8);
    }

    timer_ = nh_.createTimer(ros::Duration(0), &CascadedPidControl::TimerCallback, this, true, false);

    ROS_INFO("Cascaded PID control node initialized. Waiting for trajectory.");
  }

  void CascadedPidControl::Destroy() {
    ROS_INFO("Cascaded PID control exited.");
  }

  void CascadedPidControl::DynamicReconfigureCallback(CascadedPidConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure requested.");
    mass_ = config.mass;
    // motors should generate at least (mass * g) N force, this is the minimum
    // value of max_thrust, and maximu value of min_thrust
    if (config.max_thrust < mass_ * GRAVITY_CONST) {
      config.max_thrust = mass_ * GRAVITY_CONST;
    }
    if (config.min_thrust > mass_ * GRAVITY_CONST) {
      config.min_thrust = mass_ * GRAVITY_CONST;
    }
    min_thrust_ = config.min_thrust;
    max_thrust_ = config.max_thrust;

    kp_x_ = config.kp_x;
    kd_x_ = config.kd_x;
    max_abs_accel_x_ = config.max_abs_accel_x;
    kp_y_ = config.kp_y;
    kd_y_ = config.kd_y;
    max_abs_accel_y_ = config.max_abs_accel_y;
    kp_z_ = config.kp_z;
    kd_z_ = config.kd_z;
    kp_roll_ = config.kp_roll;
    kp_pitch_ = config.kp_pitch;
    kp_yaw_ = config.kp_yaw;

    if (config.xy_same_params) {
      config.kp_y = kp_y_ = kp_x_;
      config.kd_y = kd_y_ = kd_x_;
      config.max_abs_accel_y = max_abs_accel_y_ = max_abs_accel_x_;
    }

    if (config.rp_same_params) {
      config.kp_pitch = kp_pitch_ = kp_roll_;
    }
  }

  void CascadedPidControl::PublishDebugTopic(const mav_msgs::EigenOdometry& odometry) {
    // debug topic includes:
    //  1. set point position
    //  2. set point velocit
    //  3. set point attitude (x component: roll, y component: pitch, z component: yaw)
    geometry_msgs::Vector3 pos_setpoint;
    tf2::toMsg(set_point_.position_W, pos_setpoint);
    geometry_msgs::Vector3 vel_setpoint;
    tf2::toMsg(set_point_.velocity_W, vel_setpoint);
    geometry_msgs::Vector3 att_setpoint;
    Eigen::Vector3d set_att;
    mav_msgs::getEulerAnglesFromQuaternion(set_point_.orientation_W_B, &set_att);
    tf2::toMsg(set_att, att_setpoint);

    position_setpoint_pub_.publish(pos_setpoint);
    velocity_setpoint_pub_.publish(vel_setpoint);
    attitude_setpoint_pub_.publish(att_setpoint);

    //  4. error of position
    //  5. error of velocity
    //  6. error of attitude (x component: roll, y component: pitch, z component: yaw)
    geometry_msgs::Vector3 pos_error;
    tf2::toMsg(set_point_.position_W - odometry.position_W, pos_error);
    geometry_msgs::Vector3 vel_error;
    tf2::toMsg(set_point_.velocity_W - odometry.getVelocityWorld(), vel_error);
    geometry_msgs::Vector3 att_error;
    Eigen::Vector3d ego_att;
    mav_msgs::getEulerAnglesFromQuaternion(odometry.orientation_W_B, &ego_att);
    tf2::toMsg(set_att - ego_att, att_error);

    position_error_pub_.publish(pos_error);
    velocity_error_pub_.publish(vel_error);
    attitude_error_pub_.publish(att_error);
  }

  void CascadedPidControl::TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& ptr) {
    timer_.stop();
    traj_points_.clear();
    command_wait_time_.clear();

    if (ptr->points.size() == 0) {
      ROS_WARN("Empty trajectory.");
      return;
    }

    ROS_INFO("New trajectory arrived.");

    auto iter = ptr->points.begin();
    double last_time = iter->time_from_start.toSec();
    command_wait_time_.push_back(last_time);
    mav_msgs::EigenTrajectoryPoint eigen_traj_point;
    mav_msgs::eigenTrajectoryPointFromMsg(*iter, &eigen_traj_point);
    traj_points_.push_back(eigen_traj_point);

    for (++iter; iter != ptr->points.end(); ++iter) {
      double traj_time = iter->time_from_start.toSec();
      command_wait_time_.push_back(traj_time - last_time);
      mav_msgs::eigenTrajectoryPointFromMsg(*iter, &eigen_traj_point);
      traj_points_.push_back(eigen_traj_point);

      last_time = traj_time;
    }

    set_point_ = traj_points_.front();
    traj_points_.pop_front();

    if (!traj_points_.empty()) {
      double wait_time = command_wait_time_.front();
      command_wait_time_.pop_front();
      timer_.setPeriod(ros::Duration(wait_time));
      timer_.start();
    }

    controller_active_ = true;
  }

  void CascadedPidControl::TimerCallback(const ros::TimerEvent& e) {
    if (traj_points_.empty()) {
      ROS_INFO("All trajectory commands have beed carried out.");
      return;
    }

    set_point_ = traj_points_.front();
    traj_points_.pop_front();
    if (!traj_points_.empty()) {
      double wait_time = command_wait_time_.front();
      command_wait_time_.pop_front();
      timer_.stop();
      timer_.setPeriod(ros::Duration(wait_time));
      timer_.start();
    }
  }

}

XROS_RUNNABLE_NODE_MAIN(cascaded_pid_control::CascadedPidControl)
