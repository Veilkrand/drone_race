// This is the cheat node: it utilizes the /tf messsage to
// produce grouth truth pose information of the drone

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "base_node.hpp"

namespace cascaded_pid_control {

  template <typename T>
  class LowPassFilter {
    public:
    explicit inline LowPassFilter(const T& initial_value = 1.0, const T& tau = 1.0) 
    : initial_(initial_value), value_(initial_value), tau_(tau) { }

    inline void Reset() {
      value_ = initial_;
    }

    inline T Update(const T& measurement, double dt) {
      T alpha = static_cast<T>(dt / (dt + tau_));
      value_ = (1 - alpha) * value_ + alpha * measurement;
      return value_;
    }

    private:
    T initial_;
    T value_;
    T tau_;
  };

  class CheatOdometryNode : public xros::RunnableNode<CheatOdometryNode> {
    public:
    CheatOdometryNode(XROS_CONSTRUCTOR_DEFAULT_PARAMETERS) 
     : XROS_CONSTRUCTOR_INIT_RUNNABLE,
       tf_listener_(buffer_),
       vx_(0, 0.05),
       vy_(0, 0.05),
       vz_(0, 0.05) { }

    virtual void Init() {
      ROS_INFO("CheatNode initialized. You should not see this message when in final solution because if so, you're cheating!");      
      odometry_pub_ = private_nh_.advertise<nav_msgs::Odometry>("odometry", 1);
      last_odometry_time_ = 0.0;
    }

    void Execute() {
      geometry_msgs::TransformStamped transform_stamped;
      try {
        transform_stamped = buffer_.lookupTransform("world", "uav/imu", ros::Time(0));
      } catch (std::exception){
        ROS_INFO("Transform from uav/imu to world not available yet.");
        return;
      }
      nav_msgs::Odometry odometry;
      odometry.header.frame_id = "world";
      odometry.header.stamp = ros::Time::now();
      odometry.child_frame_id = "uav/imu";
      odometry.pose.pose.position.x = transform_stamped.transform.translation.x;
      odometry.pose.pose.position.y = transform_stamped.transform.translation.y;
      odometry.pose.pose.position.z = transform_stamped.transform.translation.z;
      odometry.pose.pose.orientation = transform_stamped.transform.rotation;
      
      double current = odometry.header.stamp.toSec();
      if (last_odometry_time_ > 0) {
	      double dt = current - last_odometry_time_;
        // note: velocities info in twist message is encoded in body frame.
        geometry_msgs::Twist twist_world;
	      twist_world.linear.x = (odometry.pose.pose.position.x - last_position_.x) / dt;
	      twist_world.linear.y = (odometry.pose.pose.position.y - last_position_.y) / dt;
	      twist_world.linear.z = (odometry.pose.pose.position.z - last_position_.z) / dt;
        
        geometry_msgs::TransformStamped transform_stamped_inverse = buffer_.lookupTransform("uav/imu", "world", ros::Time(0));
        tf2::doTransform(twist_world.linear, odometry.twist.twist.linear, transform_stamped_inverse);
      }
      odometry_pub_.publish(odometry);

      last_position_ = odometry.pose.pose.position;
      last_odometry_time_ = current;
    }

    private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Publisher odometry_pub_;

    LowPassFilter<double> vx_;
    LowPassFilter<double> vy_;
    LowPassFilter<double> vz_;

    geometry_msgs::Point last_position_;
    double last_odometry_time_;
    
  };
  
}

XROS_RUNNABLE_NODE_MAIN_AT_RATE(cascaded_pid_control::CheatOdometryNode, 60)
