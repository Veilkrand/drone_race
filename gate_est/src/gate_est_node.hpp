#ifndef _GATE_EST_GATE_EST_NODE_HPP_
#define _GATE_EST_GATE_EST_NODE_HPP_

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <sensor_msgs/CameraInfo.h>
#include <flightgoggles/IRMarkerArray.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <memory>
#include <map>
#include <deque>

#include "base_node.hpp"
#include "frame.hpp"

namespace gate_est {

  class GateEst : public xros::RunnableNode<GateEst> {
  public:
    GateEst(XROS_CONSTRUCTOR_DEFAULT_PARAMETERS)
     : XROS_CONSTRUCTOR_INIT_RUNNABLE,
       tf_listener_(buffer_),
       gate_offsets_{0} {}

    virtual void Init();

  public:
    static constexpr std::size_t NUM_GATES = 23;

  private:
    void LoadGateNominalInformation();
    void LoadInitialPose();
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ptr);
    void IRMarkerArrayCallback(const flightgoggles::IRMarkerArrayConstPtr& ptr);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& ptr);

    void BackProject(double x, double y, double depth, Eigen::Vector3d& out);
    void Pixel2Camera(double x, double y, Eigen::Vector2d& out);
    void Pixel2Camera(double x, double y, cv::Point2d& out);
    cv::Point2d Pixel2Camera(double x, double y);

    Frame::Ptr p_ref_;

    // gate offset estimation
    // each gate we need to store the x, y & z offset
    double gate_offsets_[NUM_GATES][3];

    // camera matrix stored as eigen matrix
    Eigen::Matrix3d cm_;
    
    // storing the (nominal) position of the gates (corners)
    std::vector<std::vector<Eigen::Vector3d>> gate_corners_;

    Landmark3D gates_;

    // FIXME: don't hard-coded the nearest gate here.
    std::size_t reference_gate_idx_ = 10;
    Eigen::Vector3d initial_position_;
    Eigen::Quaterniond initial_orientation_;

    // camera matrix
    cv::Mat k_;

    ros::Subscriber camera_info_sub_;
    ros::Subscriber ir_beacons_sub_;
    ros::Subscriber odometry_sub_;

    ros::Publisher corners_viz_pub_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_;
  };
  
}

#endif
