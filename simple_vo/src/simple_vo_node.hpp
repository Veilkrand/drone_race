#ifndef _SIMPLE_VO_SIMPLE_VO_NODE_HPP_
#define _SIMPLE_VO_SIMPLE_VO_NODE_HPP_

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <sensor_msgs/CameraInfo.h>
#include <flightgoggles/IRMarkerArray.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <memory>
#include <map>

#include "base_node.hpp"

#include "frame.hpp"

namespace simple_vo {

  class SimpleVo : public xros::RunnableNode<SimpleVo> {
  public:
    XROS_DECLARE_RUNNABLE_NODE_CONSTRUCTOR(SimpleVo)

    virtual void Init();

  private:
    void LoadGateNominalInformation();
    void LoadInitialPose();
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ptr);
    void IRMarkerArrayCallback(const flightgoggles::IRMarkerArrayConstPtr& ptr);

    void BackProject(double x, double y, double depth, Eigen::Vector3d& out);
    void Pixel2Camera(double x, double y, Eigen::Vector2d& out);
    void Pixel2Camera(double x, double y, cv::Point2d& out);    

    double scale_;
    bool initialized_;

    std::shared_ptr<Frame> p_ref_;
    
    // storing the (nominal) position of the gates (corners)
    std::vector<std::vector<Eigen::Vector3d>> gate_corners_;
    
    // FIXME: don't hard-coded the nearest gate here.
    std::size_t reference_gate_idx_ = 10;
    Eigen::Vector3d initial_position_;
    Eigen::Quaterniond initial_orientation_;

    // camera matrix
    cv::Mat k_;
    
    mav_msgs::EigenOdometry odometry_;

    ros::Subscriber camera_info_sub_;
    ros::Subscriber ir_beacons_sub_;
  };
  
}

#endif
