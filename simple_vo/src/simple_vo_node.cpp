#include "simple_vo_node.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include <cstring>
#include <string>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace {
  inline std::size_t GetLandmarkKey(std::size_t gate_idx, std::size_t corner_idx) {
    return 10000 + gate_idx * 100 + corner_idx;
  }
}

namespace simple_vo {

  void SimpleVo::BackProject(double x, double y, double depth, Eigen::Vector3d& out) {
    const double cx = k_.at<double>(0, 2);
    const double cy = k_.at<double>(1, 2);
    const double fx = k_.at<double>(0, 0);
    const double fy = k_.at<double>(1, 1);
    out << (x - cx) * depth / fx,
           (y - cy) * depth / fy,
           depth;
  }

  void SimpleVo::Pixel2Camera(double x, double y, Eigen::Vector2d& out) {
    const double cx = k_.at<double>(0, 2);
    const double cy = k_.at<double>(1, 2);
    const double fx = k_.at<double>(0, 0);
    const double fy = k_.at<double>(1, 1);
    out << (x - cx) / fx,
           (y - cy) / fy;
  }

  void SimpleVo::Pixel2Camera(double x, double y, cv::Point2d& out) {
    const double cx = k_.at<double>(0, 2);
    const double cy = k_.at<double>(1, 2);
    const double fx = k_.at<double>(0, 0);
    const double fy = k_.at<double>(1, 1);
    out.x = (x - cx) / fx;
    out.y = (y - cy) / fy;
  }

  Eigen::VectorXd SimpleVo::EstimateDepthOfGate(std::size_t gate_idx, std::vector<cv::Point2d> corners_2d) {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::solvePnP(gate_corners_rel_[gate_idx], corners_2d, k_, cv::Mat(), rvec, tvec);
    // rvec/tvec contains transformation from object coordinate to
    // camera coordinate. because the way gate_corners_rel_ are arranged,
    // the can be consider as "gate local coordinate" with the first 
    // corner of each gate as the origin.
    //
    // so rvec/tvec can give us information about the distance from
    // the camera principal point to the first corner of the gate,
    // then we can know the distance to other corners to.
    //
    // with this information, we can estimate depth information of
    // each corner of the gate.
    cv::Mat r;
    cv::Rodrigues(rvec, r);

    Eigen::VectorXd result(4);
    for (std::size_t i = 0; i < 4; ++i) {
      ROS_INFO_STREAM(cv::norm(gate_corners_rel_[gate_idx][i] - gate_corners_rel_[gate_idx][0]));
      cv::Mat pt = (cv::Mat_<double>(3, 1)<<gate_corners_rel_[gate_idx][i].x,
		    gate_corners_rel_[gate_idx][i].y,
		    gate_corners_rel_[gate_idx][i].z);
      double d = cv::norm(r * pt + tvec);
      Eigen::Vector3d cam_h;
      BackProject(corners_2d[i].x, corners_2d[i].y, 1.0, cam_h);
      result[i] = d / cam_h.norm();
    }

    return result;
  }

  void SimpleVo::IRMarkerArrayCallback(const flightgoggles::IRMarkerArrayConstPtr& ptr) {
    if (!initialized_) {
      char buf[64];
      std::sprintf(buf, "Gate%zu", reference_gate_idx_);
      std::string target_gate(buf);

      p_ref_ = std::make_shared<Frame>();
      p_ref_->timestamp = ptr->header.stamp.toSec();
      // p_ref_->t_c_w = Sophus::SE3d(initial_orientation_.inverse(), -initial_position_);
      p_ref_->t_c_w = Sophus::SE3d();

      std::vector<cv::Point2d> ref_gate_corners(4);
      auto iter = ptr->markers.begin();
      while (iter != ptr->markers.end()) {
        if (iter->landmarkID.data == target_gate) {
          std::size_t idx = std::atoi(iter->markerID.data.c_str());
	  ref_gate_corners[idx-1].x = iter->x;
	  ref_gate_corners[idx-1].y = iter->y;

	  StoreLandmark(p_ref_->landmarks_2d, reference_gate_idx_, idx, Eigen::Vector2d(iter->x, iter->y));
        }
        ++iter;
      }

      const Eigen::VectorXd depth = EstimateDepthOfGate(reference_gate_idx_,
							ref_gate_corners);
      ROS_INFO_STREAM("Depth:"<<depth);

      for (std::size_t i = 1; i <= 4; ++i) {
	Eigen::Vector3d pt_3d;
	const Eigen::Vector2d& pt_2d = GetLandmark(p_ref_->landmarks_2d, reference_gate_idx_, i);
	
	BackProject(pt_2d[0], pt_2d[1], depth[i-1], pt_3d);

	ROS_INFO_STREAM("Gate corner "<<i<<" coordinate in camera frame:"<<pt_3d);
	
	if (i > 1) {
	  ROS_INFO_STREAM((pt_3d - GetLandmark(p_ref_->landmarks_3d, reference_gate_idx_, 1)).norm());
	}
	StoreLandmark(p_ref_->landmarks_3d, reference_gate_idx_, i, pt_3d);
      }
      
      initialized_ = true;

      ROS_INFO("Initialized");
      return;
    }

    return;

    auto p_curr = std::make_shared<Frame>();
    p_curr->timestamp = ptr->header.stamp.toSec();
    
    std::vector<cv::Point3d> pts_3d;
    std::vector<cv::Point2d> pts_2d;
    auto iter = ptr->markers.begin();
    while (iter != ptr->markers.end()) {
      std::size_t gate_idx = std::atoi(iter->landmarkID.data.substr(4).c_str());
      std::size_t idx = std::atoi(iter->markerID.data.c_str());

      StoreLandmark(p_curr->landmarks_2d, gate_idx, idx, Eigen::Vector2d(iter->x, iter->y));

      if (HasLandmark(p_ref_->landmarks_3d, gate_idx, idx)) {
	cv::Point3d p3d;
	auto& landmark = GetLandmark(p_ref_->landmarks_3d, gate_idx, idx);
	EigenVector2CvPoint(landmark, p3d);
	pts_3d.push_back(p3d);
	StoreLandmark(p_curr->landmarks_3d, gate_idx, idx, landmark);
	
	cv::Point2d p2d(iter->x, iter->y);
        pts_2d.push_back(p2d);
      } else {
	// ROS_INFO_STREAM("New landmark: "<<GetLandmarkKey(gate_idx, idx));
      }

      ++iter;
    }

    //    ROS_INFO_STREAM(pts_3d);
    //    ROS_INFO_STREAM(pts_2d);
    
    cv::Mat rvec;
    cv::Mat tvec;
    // cv::Mat inliners;
    // cv::solvePnPRansac(pts_3d, pts_2d, k_, cv::Mat(), rvec, tvec, false, 100, 4.0f, 1, inliners);
    // if (inliners.rows < 4) {
    //   return;
    // }
    cv::solvePnP(pts_3d, pts_2d, k_, cv::Mat(), rvec, tvec);
    // if movement is too small, ignore this frame
    if (std::sqrt(std::pow(tvec.at<double>(0), 2) +
		  std::pow(tvec.at<double>(1), 2) +
		  std::pow(tvec.at<double>(2), 2)) < 1e-6) {
      return;
    }
    
    // rvec and tvec are transformation from reference camera coordinates to current camera coordinates, t_c_r
    // each reference frame contains a transformation from world coordinates to camera coordinates, t_c_w, and in this context, t_c_w of reference frame should be denoted as t_r_w
    // so t_c_w for the new frame is: t_c_w = t_c_r * t_r_w
    Sophus::SE3d t_c_r(Sophus::SO3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
		       Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
    p_curr->t_c_w = t_c_r * p_ref_->t_c_w;

    //    ROS_INFO_STREAM(p_ref_->t_c_w.translation());
    ROS_INFO_STREAM("rvec: "<<rvec.at<double>(0)<<" "<<rvec.at<double>(1)<<" "<<rvec.at<double>(2));
    ROS_INFO_STREAM("tvec: "<<tvec.at<double>(0)<<" "<<tvec.at<double>(1)<<" "<<tvec.at<double>(2));        
    //    ROS_INFO_STREAM(p_curr->t_c_w.translation());

    // use triangulation for estimating the depth of landmarks
    if (p_curr->landmarks_2d.size() > 0) {
      cv::Mat r;
      cv::Rodrigues(rvec, r);
      //      ROS_INFO_STREAM(rvec);
      // ROS_INFO_STREAM(r);
      
      cv::Mat p0 = cv::Mat::eye(3, 4, CV_64F);
      cv::Mat p1(3, 4, CV_64F);
      p1(cv::Range::all(), cv::Range(0, 3)) = r * 1.0;
      p1.col(3) = tvec * 1.0;
      // ROS_INFO_STREAM(p0);
      // ROS_INFO_STREAM(p1);

      cv::Mat q;
      std::vector<std::size_t> keys;
      std::vector<cv::Point2d> first;
      std::vector<cv::Point2d> second;
      auto iter = p_curr->landmarks_2d.begin();
      while (iter != p_curr->landmarks_2d.end()) {
    	std::size_t k = iter->first;
    	if (HasLandmark(p_ref_->landmarks_2d, k)) {
	  const Eigen::Vector2d& ef = GetLandmark(p_ref_->landmarks_2d, k);
	  const Eigen::Vector2d& es = iter->second;
    	  cv::Point2d f;
    	  cv::Point2d s;
	  Pixel2Camera(ef[0], ef[1], f);
	  Pixel2Camera(es[0], es[1], s);
    	  keys.push_back(k);
    	  first.push_back(f);
    	  second.push_back(s);
    	}
    	++iter;
      }
      
      if (keys.size() > 0) {
    	cv::triangulatePoints(p0, p1, first, second, q);
	ROS_INFO_STREAM(q);
	q.row(0) /= q.row(3);
	q.row(1) /= q.row(3);
    	q.row(2) /= q.row(3);
	ROS_INFO_STREAM(q);	
    	ROS_INFO_STREAM("Depth of "<<keys.size()<<" landmarks has been determined by triangulation");
    	for (std::size_t i = 0; i < keys.size(); ++i) {
    	  std::size_t k = keys[i];
    	  ROS_INFO_STREAM("depth of "<<k<<": "<<q.at<double>(2, i));
          Eigen::Vector3d pt_3d;
    	  const Eigen::Vector2d& pt_2d = GetLandmark(p_curr->landmarks_2d, k);
          BackProject(pt_2d[0], pt_2d[1], q.at<double>(2, i), pt_3d);
    	  StoreLandmark(p_curr->landmarks_3d, k, pt_3d);
    	}
      }
    }

    p_ref_ = p_curr;
    
    //    ROS_INFO_STREAM("Num inliners: "<<inliners.rows);
    
    // odometry_.position_W[0] = tvec.at<double>(0);
    // odometry_.position_W[1] = tvec.at<double>(1);
    // odometry_.position_W[2] = tvec.at<double>(2);

    // Eigen::Vector3d v;
    // v << rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2);
    // double theta = v.norm();
    // v /= theta;
    // double s_half = std::sin(theta / 2);
    // double c_half = std::cos(theta / 2);
    // odometry_.orientation_W_B.x() = s_half * v[0];
    // odometry_.orientation_W_B.y() = s_half * v[1];
    // odometry_.orientation_W_B.z() = s_half * v[2];
    // odometry_.orientation_W_B.w() = c_half;

    // ROS_INFO_STREAM("Rotation: " << r << std::endl << "Translation: "<< t << std::endl);
  }

  void SimpleVo::Init() {
    // read in course definition and initial pose, then establish a
    // reference frame base on these information
    LoadGateNominalInformation();
    LoadInitialPose();
    initialized_ = false;
    reference_gate_idx_ = 10;
    
    camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/uav/camera/left/camera_info", 1, &SimpleVo::CameraInfoCallback, this);
    ir_beacons_sub_ = nh_.subscribe<flightgoggles::IRMarkerArray>("/uav/camera/left/ir_beacons", 1, &SimpleVo::IRMarkerArrayCallback, this);
  }

  void SimpleVo::LoadGateNominalInformation() {
    gate_corners_.clear();
    gate_corners_.push_back(std::vector<Eigen::Vector3d>());

    constexpr std::size_t num_gates = 23;

    char buf[256];

    for (std::size_t i = 1; i < num_gates; ++i) {
      std::sprintf(buf, "/uav/Gate%zu/nominal_location", i);
      XmlRpc::XmlRpcValue corners;
      nh_.getParam(buf, corners);
      std::vector<Eigen::Vector3d> cs;
      std::vector<cv::Point3d> cs_rel;
      for (std::size_t j = 0; j < 4; ++j) {
        cs.push_back(Eigen::Vector3d(corners[j][0], corners[j][1], corners[j][2]));
	if (j == 0) {
	  // 0, 0, 0 for the first corner
	  cs_rel.push_back(cv::Point3d(0, 0, 0));
	} else {
	  // position relative to the first corner for the others
	  const Eigen::Vector3d v = cs[j] - cs[0];
	  cs_rel.push_back(cv::Point3d(v[0], v[1], v[2]));
	}
      }

      gate_corners_.push_back(cs);
      gate_corners_rel_.push_back(cs_rel);
    }
    ROS_INFO("Nominal gates information loaded.");
  }

  void SimpleVo::LoadInitialPose() {
    std::vector<double> pose;
    nh_.getParam("/uav/flightgoggles_uav_dynamics/init_pose", pose);
    initial_position_ << pose[0], pose[1], pose[2];
    initial_orientation_ = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
  }

  void SimpleVo::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ptr) {
    k_ = (cv::Mat_<double>(3, 3) 
            << ptr->K[0], ptr->K[1], ptr->K[2],
               ptr->K[3], ptr->K[4], ptr->K[5],
               ptr->K[6], ptr->K[7], ptr->K[8]);
  }

}

XROS_RUNNABLE_NODE_MAIN(simple_vo::SimpleVo)
