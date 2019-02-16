#ifndef _CASCADED_PID_CONTROL_CASCADED_PID_CONTROL_NODE_
#define _CASCADED_PID_CONTROL_CASCADED_PID_CONTROL_NODE_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <cascaded_pid_control/CascadedPidConfig.h>
#include "base_node.hpp"

namespace cascaded_pid_control {

  constexpr double GRAVITY_CONST = 9.8;

  class CascadedPidControl : public xros::RunnableNode<CascadedPidControl> {
    public:
    XROS_DECLARE_RUNNABLE_NODE_CONSTRUCTOR(CascadedPidControl)

    virtual void Init();

    virtual void Destroy();

    private:
    void DynamicReconfigureCallback(CascadedPidConfig &config, uint32_t level);

    void OdometryCallback(const nav_msgs::OdometryConstPtr &ptr);

    /**
     * @param pose current pose in NWU
     * @param twist current velocity in NWU
     * @param pose_cmd commanded pose in NWU. only the z component is used.
     * @param twist_cmd velocity in NWU. only the z component is used.
     * @param accel_ff feed forward acceleration in NWU. only the z component is used.
     * @param dt time difference of measurements
     * @return collective thrust commanded. since thrust is always measured in 
     *         body frame, the x & y component of it is always zero.
     */
    geometry_msgs::Vector3 AltitudeControl(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Twist &twist,
      const geometry_msgs::Pose &pose_cmd,
      const geometry_msgs::Twist &twist_cmd,
      const geometry_msgs::Vector3 &accel_ff,
      double dt);

    /**
     * @param pose current pose in NWU
     * @param twist current velocity in NWU
     * @param pose_cmd commanded pose in NWU. z component is ignored.
     * @param twist_cmd velocity in NWU. z component is ignored.
     * @param accel_ff feed forward acceleration in NWU. z component is ignored.
     * @param dt time difference of measurements
     * @return lateral acceleration command
     */
    geometry_msgs::Vector3 LateralPositionControl(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Twist &twist,
      const geometry_msgs::Pose &pose_cmd,
      const geometry_msgs::Twist &twist_cmd,
      const geometry_msgs::Vector3 &accel_ff,
      double dt);


    geometry_msgs::Vector3 AttitudeControl(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Vector3 &accel_cmd,
      const geometry_msgs::Vector3 &thrust,
      double dt);

    private:
    // parameters
    double mass_;
    double kp_x_;
    double kd_x_;
    double kp_y_;
    double kd_y_;
    double kp_z_;
    double kd_z_;
    double kp_roll_;
    double kp_pitch_;
    double kp_yaw_;

    // member variables
    double last_odometry_time_;
    dynamic_reconfigure::Server<CascadedPidConfig> server_;
    ros::Publisher rate_thrust_pub_;
    ros::Subscriber odometry_sub_;

  };

}

#endif
