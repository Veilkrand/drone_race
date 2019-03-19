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

    void UpdateEstimatedPositionsOfGates(std::shared_ptr<Frame> frame, cv::Mat rvec=cv::Mat(), cv::Mat tvec=cv::Mat());
    void UpdateEstimatedPositionOfSingleGate(std::shared_ptr<Frame> frame,
					     const std::vector<cv::Point2d>& pts_2d,
					     const std::vector<cv::Point3d>& pts_3d,
					     const std::vector<std::size_t>& keys,
					     cv::Mat rvec=cv::Mat(),
					     cv::Mat tvec=cv::Mat());

    double scale_;
    bool initialized_;

    std::shared_ptr<Frame> p_ref_;
    
    // storing the (nominal) position of the gates (corners)
    std::vector<std::vector<Eigen::Vector3d>> gate_corners_;

    // storing gates coordinates w.r.t. the first corner of each gate.
    // used in estimating depth information of gate
    std::vector<std::vector<cv::Point3d>> gate_corners_rel_;

    // FIXME: don't hard-coded the nearest gate here.
    std::size_t reference_gate_idx_ = 10;
    Eigen::Vector3d initial_position_;
    Eigen::Quaterniond initial_orientation_;

    // camera matrix
    cv::Mat k_;
    
    mav_msgs::EigenOdometry odometry_;

    ros::Subscriber camera_info_sub_;
    ros::Subscriber ir_beacons_sub_;

    ros::Publisher corners_viz_pub_;
  };
  
}

#endif
