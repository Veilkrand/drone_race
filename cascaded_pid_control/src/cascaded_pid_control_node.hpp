#ifndef _CASCADED_PID_CONTROL_CASCADED_PID_CONTROL_NODE_
#define _CASCADED_PID_CONTROL_CASCADED_PID_CONTROL_NODE_

#include <deque>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <dynamic_reconfigure/server.h>
#include <cascaded_pid_control/CascadedPidConfig.h>
#include "base_node.hpp"

namespace cascaded_pid_control {

  constexpr double GRAVITY_CONST = 9.8;
  constexpr double PI = 3.14159265358979323846;

  class CascadedPidControl : public xros::RunnableNode<CascadedPidControl> {
    public:
    XROS_DECLARE_RUNNABLE_NODE_CONSTRUCTOR(CascadedPidControl)

    virtual void Init();

    virtual void Destroy();

    /**
     * @param current current odometry in NWU
     * @param cmd commanded point in NWU. only the z component is used.
     * @param accel_ff feed forward acceleration in NWU. only the z component is used.
     * @param dt time difference of measurements
     * @return collective thrust
     */
    double AltitudeControl(
      const mav_msgs::EigenOdometry& current,
      const mav_msgs::EigenTrajectoryPoint& cmd,
      const Eigen::Vector3d& accel_ff,
      double dt);

    /**
     * @param current current odometry in NWU
     * @param cmd commanded point in NWU. z component is ignored.
     * @param accel_ff feed forward acceleration in NWU. z component is ignored.
     * @param dt time difference of measurements
     * @return lateral acceleration command in NWU (z component is always 0)
     */
    Eigen::Vector3d LateralPositionControl(
      const mav_msgs::EigenOdometry& current,
      const mav_msgs::EigenTrajectoryPoint& cmd,
      const Eigen::Vector3d& accel_ff,
      double dt);

    /**
     * @param current current odometry in NWU
     * @param accel_cmd linear acceleration command in NWU
     * @param thrust thrust command (always in body frame)
     * @param dt time difference of measurements
     * @return body rate command (except yaw rate. always 0)
     */
    Eigen::Vector3d AttitudeControl(
      const mav_msgs::EigenOdometry& current,
      const Eigen::Vector3d& accel_cmd,
      double thrust_cmd,
      double dt);

    /**
     * @param current current odometry in NWU
     * @param yaw_cmd yaw command
     * @param dt time difference of measurements
     * @return yaw rate command
     */
    double YawControl(
      const mav_msgs::EigenOdometry& current,
      double yaw_cmd,
      double dt);

    private:
    void DynamicReconfigureCallback(CascadedPidConfig &config, uint32_t level);

    void OdometryCallback(const nav_msgs::OdometryConstPtr &ptr);
    void TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& ptr);
    void TrajectoryCallbackNew(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& ptr);
    void TrajectoryCallbackStartFromNearest(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& ptr);    

    void TimerCallback(const ros::TimerEvent& e);

    void PublishDebugTopic(const mav_msgs::EigenOdometry& odometry);

    void SetNextPoint(const mav_msgs::EigenTrajectoryPoint& point);

    private:
    // parameters
    double mass_;
    double min_thrust_;
    double max_thrust_;
    double kp_x_;
    double kd_x_;
    double max_abs_accel_x_;
    double kp_y_;
    double kd_y_;
    double max_abs_accel_y_;
    double kp_z_;
    double kd_z_;
    double kp_roll_;
    double kp_pitch_;
    double kp_yaw_;

    // other member variables
    mav_msgs::EigenOdometry last_odometry_;
    double last_odometry_time_;
    dynamic_reconfigure::Server<CascadedPidConfig> server_;
    ros::Publisher rate_thrust_pub_;
    ros::Subscriber odometry_sub_;

    ros::Subscriber trajectory_sub_;
    // timer for picking up trajectory point
    ros::Timer timer_;
    // dequeue storing issued trajectory points
    std::deque<mav_msgs::EigenTrajectoryPoint> traj_points_;
    std::deque<double> command_wait_time_;

    // current trajectory point
    mav_msgs::EigenTrajectoryPoint set_point_;
    bool controller_active_;

    bool publish_debug_topic_;
    // debug topic
    ros::Publisher position_setpoint_pub_;
    ros::Publisher velocity_setpoint_pub_;
    ros::Publisher attitude_setpoint_pub_;
    ros::Publisher position_error_pub_;
    ros::Publisher velocity_error_pub_;
    ros::Publisher attitude_error_pub_;
    ros::Publisher target_pose_pub_;

    Eigen::Vector3d default_ff_;
  };

}

#endif
