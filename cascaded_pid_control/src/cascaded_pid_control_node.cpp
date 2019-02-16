#include <cmath>

#include <ros/ros.h>

#include <mav_msgs/RateThrust.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cascaded_pid_control_node.hpp"

namespace cascaded_pid_control {

  inline Eigen::Quaterniond EigenQuat(const geometry_msgs::Quaternion &msg) {
    return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
  }

  inline Eigen::Matrix3d EigenRotMat(const geometry_msgs::Quaternion &msg) {
    return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z).toRotationMatrix();
  }

  inline Eigen::Vector3d EigenVec3(const geometry_msgs::Vector3& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
  }
  
  inline Eigen::Vector3d EigenVec3(const geometry_msgs::Point& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
  }

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

  geometry_msgs::Vector3 CascadedPidControl::AltitudeControl(
    const geometry_msgs::Pose &pose,
    const geometry_msgs::Twist &twist,
    const geometry_msgs::Pose &pose_cmd,
    const geometry_msgs::Twist &twist_cmd,
    const geometry_msgs::Vector3 &accel_ff,
    double dt) {

      Eigen::Matrix3d rot_mat = EigenRotMat(pose.orientation);

      double dz = pose_cmd.position.z - pose.position.z;
      double dvz = twist_cmd.linear.z - twist.linear.z;
      double z_dot_dot = kp_z_ * dz + kd_z_ * dvz + accel_ff.z;

      geometry_msgs::Vector3 result;
      result.z = z_dot_dot * mass_ / rot_mat(2, 2);

      // clamp value
      result.z = std::max(0.0, result.z);

      // ROS_INFO_STREAM(dz<<" "<<z_dot_dot<<" "<<result.z);

      return result;
  }

  geometry_msgs::Vector3 CascadedPidControl::LateralPositionControl(
    const geometry_msgs::Pose &pose,
    const geometry_msgs::Twist &twist,
    const geometry_msgs::Pose &pose_cmd,
    const geometry_msgs::Twist &twist_cmd,
    const geometry_msgs::Vector3 &accel_ff,
    double dt) {

    double err_x = pose_cmd.position.x - pose.position.x;
    double err_y = pose_cmd.position.y - pose.position.y;

    double err_vx = twist_cmd.linear.x - twist.linear.x;
    double err_vy = twist_cmd.linear.y - twist.linear.y;
    
    geometry_msgs::Vector3 accel_cmd;
    accel_cmd.x = Constrain(kp_x_ * err_x + kd_x_ * err_vx + accel_ff.x, -1, 1);
    accel_cmd.y = Constrain(kp_y_ * err_y + kd_y_ * err_vy + accel_ff.y, -1, 1);

    // ROS_INFO_STREAM("x: err: "<<err_x<<" cmd: "<<accel_cmd.x);
    // ROS_INFO_STREAM("y: err: "<<err_y<<" cmd: "<<accel_cmd.y);
    return accel_cmd;
  }

  geometry_msgs::Vector3 CascadedPidControl::AttitudeControl(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Vector3 &accel_cmd,
      const geometry_msgs::Vector3 &thrust,
      double dt) {
    Eigen::Matrix3d rot_mat = EigenRotMat(pose.orientation);

    double bx = rot_mat(0, 2);
    double by = rot_mat(1, 2);
    
    double c = thrust.z / mass_;
    double bx_cmd = accel_cmd.x / c;
    double by_cmd = accel_cmd.y / c;

    double bx_dot = kp_pitch_ * (bx_cmd - bx);
    double by_dot = kp_roll_ * (by_cmd - by);

    double k = 1.0 / rot_mat(2, 2);
    geometry_msgs::Vector3 pq_rate_cmd;
    pq_rate_cmd.x = k * (rot_mat(1, 0) * bx_dot - rot_mat(0, 0) * by_dot);
    pq_rate_cmd.y = k * (rot_mat(1, 1) * bx_dot - rot_mat(0, 1) * by_dot);

    // ROS_INFO_STREAM("p rate: "<<pq_rate_cmd.x<<"  y accel: "<<accel_cmd.y);
    // ROS_INFO_STREAM("q rate: "<<pq_rate_cmd.y<<"  x accel: "<<accel_cmd.x);

    return pq_rate_cmd;
  }

  void CascadedPidControl::OdometryCallback(const nav_msgs::OdometryConstPtr &ptr) {
    const geometry_msgs::Pose &current_pose = ptr->pose.pose;
    const geometry_msgs::Twist &current_twist = ptr->twist.twist;
    geometry_msgs::Pose pose_cmd;
    pose_cmd.orientation.w = 1;
    pose_cmd.position.z = 3;
    geometry_msgs::Twist twist_cmd;
    geometry_msgs::Vector3 accel_ff;
    accel_ff.x = 0;
    accel_ff.y = 0;
    accel_ff.z = GRAVITY_CONST;

    double current_time = ptr->header.stamp.toSec();
    double dt = current_time - last_odometry_time_;

    geometry_msgs::Vector3 thrust = AltitudeControl(current_pose, current_twist, pose_cmd, twist_cmd, accel_ff, dt);
    geometry_msgs::Vector3 accel_cmd = LateralPositionControl(current_pose, current_twist, pose_cmd, twist_cmd, accel_ff, dt);
    geometry_msgs::Vector3 pq_rate_cmd = AttitudeControl(current_pose, accel_cmd, thrust, dt);

    // control command is published as mav_msgs/RateThrust
    mav_msgs::RateThrust rate_thrust;
    rate_thrust.header.stamp = ros::Time::now();
    rate_thrust.header.frame_id = "uav/imu";
    rate_thrust.thrust = thrust;
    rate_thrust.angular_rates = pq_rate_cmd;
    rate_thrust_pub_.publish(rate_thrust);

    last_odometry_time_ = current_time;
  }

  void CascadedPidControl::Init() {
    dynamic_reconfigure::Server<CascadedPidConfig>::CallbackType cb;
    cb = boost::bind(&CascadedPidControl::DynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(cb);

    rate_thrust_pub_ = private_nh_.advertise<mav_msgs::RateThrust>("rateThrust", 1);
    odometry_sub_ = private_nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &CascadedPidControl::OdometryCallback, this);

    ROS_INFO("Cascaded PID control node initialized.");
  }

  void CascadedPidControl::Destroy() {
    ROS_INFO("Cascaded PID control exited.");
  }

  void CascadedPidControl::DynamicReconfigureCallback(CascadedPidConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure requested.");
    mass_ = config.mass;
    kp_x_ = config.kp_x;
    kd_x_ = config.kd_x;
    kp_y_ = config.kp_y;
    kd_y_ = config.kd_y;
    kp_z_ = config.kp_z;
    kd_z_ = config.kd_z;
    kp_roll_ = config.kp_roll;
    kp_pitch_ = config.kp_pitch;
    kp_yaw_ = config.kp_yaw;

    if (config.xy_same_params) {
      config.kp_y = kp_y_ = kp_x_;
      config.kd_y = kd_y_ = kd_x_;
    }

    if (config.rp_same_params) {
      config.kp_pitch = kp_pitch_ = kp_roll_;
    }
  }

}

XROS_RUNNABLE_NODE_MAIN(cascaded_pid_control::CascadedPidControl)
