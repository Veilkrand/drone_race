#include "simple_vo_node.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include <visualization_msgs/Marker.h>

#include <cstring>
#include <string>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>


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

  void SimpleVo::UpdateEstimatedPositionsOfGates(std::shared_ptr<Frame> frame) {
    std::vector<cv::Point2d> pts_2d;
    std::vector<cv::Point3d> pts_3d;
    std::vector<std::size_t> keys;
    std::size_t previous_gate = 0;
    auto iter = frame->landmarks_2d.begin();
    while (iter != frame->landmarks_2d.end()) {
      std::size_t gate_idx = GetGateIndex(iter->first);
      std::size_t corner_idx = GetCornerIndex(iter->first);
      if (previous_gate != gate_idx) {
        if (pts_2d.size() >= 3) {
	        UpdateEstimatedPositionOfSingleGate(frame, pts_2d, pts_3d, keys);
        }
	      previous_gate = gate_idx;
	      pts_2d.clear();
	      pts_3d.clear();
	      keys.clear();
      }

      cv::Point2d p2d(iter->second[0], iter->second[1]);
      pts_2d.push_back(p2d);
      pts_3d.push_back(gate_corners_rel_[gate_idx][corner_idx-1]);
      keys.push_back(iter->first);
      ++iter;
    }

    if (pts_2d.size() >= 3) {
      UpdateEstimatedPositionOfSingleGate(frame, pts_2d, pts_3d, keys);
    }
  }

  void SimpleVo::UpdateEstimatedPositionOfSingleGate(std::shared_ptr<Frame> frame,
						     const std::vector<cv::Point2d>& pts_2d,
						     const std::vector<cv::Point3d>& pts_3d,
						     const std::vector<std::size_t>& keys) {
    ROS_INFO_STREAM("Keys: "<<keys[0]);
    cv::Mat rvec;
    cv::Mat tvec;
    if (pts_2d.size() == 3) {
      ROS_INFO_STREAM("P3P");
      std::size_t gate_idx = GetGateIndex(keys[0]);

      cv::Mat_<double> p3d(3, 3);
      cv::Mat_<double> p2d(3, 2);
      std::vector<cv::Mat> rvecs;
      std::vector<cv::Mat> tvecs;

      int num_solutions = cv::solveP3P(pts_3d, pts_2d, k_, cv::Mat(), rvecs, tvecs, cv::SOLVEPNP_AP3P);
      ROS_INFO_STREAM("Solutions: "<<num_solutions);
      for (int s = 0; s < num_solutions; ++s) {
        rvec = rvecs[s];
        tvec = tvecs[s];
        cv::Mat r;
        cv::Rodrigues(rvec, r);
        std::size_t detected = 0;
        for (std::size_t i = 0; i < pts_2d.size(); ++i) {
          //      ROS_INFO_STREAM(cv::norm(gate_corners_rel_[gate_idx][i] - gate_corners_rel_[gate_idx][0]));
          detected += GetCornerIndex(keys[i]);
          cv::Mat pt = (cv::Mat_<double>(3, 1) << pts_3d[i].x, pts_3d[i].y, pts_3d[i].z);
          double d = cv::norm(r * pt + tvec);
          Eigen::Vector3d cam_h;
          BackProject(pts_2d[i].x, pts_2d[i].y, 1.0, cam_h);
          double depth = d / cam_h.norm();
          cam_h *= depth;
          ROS_INFO_STREAM("Solution "<< s << ", pt: "<<cam_h);
          StoreLandmark(frame->landmarks_3d, keys[i], cam_h);
        }
        std::size_t missing = (1+2+3+4) - detected;
        std::size_t ref = missing == 1 ? 2 : 1;
        const cv::Point3d& shift = gate_corners_rel_[gate_idx][missing - 1] - gate_corners_rel_[gate_idx][ref - 1];
        cv::Point3d missing_pt;
        missing_pt.x = tvec.at<double>(0) + shift.x;
        missing_pt.y = tvec.at<double>(1) + shift.y;
        missing_pt.z = tvec.at<double>(2) + shift.z;
        std::vector<cv::Point3d> all_corners(pts_3d);
        all_corners.insert(all_corners.begin() + missing - 1, missing_pt);
        // calculate reprojection error.
        std::vector<cv::Point2d> reprojected;
        cv::projectPoints(all_corners, rvec, tvec, k_, cv::Mat(), reprojected);
        ROS_INFO_STREAM(reprojected[0]);
        ROS_INFO_STREAM(reprojected[1]);
        ROS_INFO_STREAM(reprojected[2]);
        ROS_INFO_STREAM(reprojected[3]);
      } 
    } else {
      cv::solvePnP(pts_3d, pts_2d, k_, cv::Mat(), rvec, tvec);
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

      std::vector<int> initial_guess;
      for (std::size_t i = 0; i < pts_2d.size(); ++i) {
        //      ROS_INFO_STREAM(cv::norm(gate_corners_rel_[gate_idx][i] - gate_corners_rel_[gate_idx][0]));
        cv::Mat pt = (cv::Mat_<double>(3, 1) << pts_3d[i].x, pts_3d[i].y, pts_3d[i].z);
        double d = cv::norm(r * pt + tvec);
        Eigen::Vector3d cam_h;
        BackProject(pts_2d[i].x, pts_2d[i].y, 1.0, cam_h);
        double depth = d / cam_h.norm();
        cam_h *= depth;
        StoreLandmark(frame->landmarks_3d, keys[i], cam_h);
      }
    }
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
      p_ref_ = std::make_shared<Frame>();
      p_ref_->timestamp = ptr->header.stamp.toSec();
      // p_ref_->t_c_w = Sophus::SE3d(initial_orientation_.inverse(), -initial_position_);
      p_ref_->t_c_w = Sophus::SE3d();

      std::vector<cv::Point2d> ref_gate_corners(4);
      auto iter = ptr->markers.begin();
      while (iter != ptr->markers.end()) {
        std::size_t gate_idx = std::atoi(iter->landmarkID.data.substr(4).c_str());
        std::size_t idx = std::atoi(iter->markerID.data.c_str());
	      ref_gate_corners[idx-1].x = iter->x;
	      ref_gate_corners[idx-1].y = iter->y;

	      StoreLandmark(p_ref_->landmarks_2d,
		    gate_idx, idx,
		    Eigen::Vector2d(iter->x, iter->y));
        ++iter;
      }

      UpdateEstimatedPositionsOfGates(p_ref_);
      
      initialized_ = true;

      ROS_INFO("Initialized");
      return;
    }

    auto p_curr = std::make_shared<Frame>();
    p_curr->timestamp = ptr->header.stamp.toSec();
    
    std::vector<cv::Point3d> pts_3d;
    std::vector<cv::Point2d> pts_2d;
    auto iter = ptr->markers.begin();
    while (iter != ptr->markers.end()) {
      std::size_t gate_idx = std::atoi(iter->landmarkID.data.substr(4).c_str());
      std::size_t idx = std::atoi(iter->markerID.data.c_str());

      StoreLandmark(p_curr->landmarks_2d, gate_idx, idx, Eigen::Vector2d(iter->x, iter->y));
      ++iter;
    }

    UpdateEstimatedPositionsOfGates(p_curr);

    visualization_msgs::Marker corners_marker;
    corners_marker.header.stamp = ros::Time::now();
    corners_marker.header.frame_id = "uav/camera/left";
    corners_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    corners_marker.action = visualization_msgs::Marker::ADD;
    corners_marker.id = 1;
    corners_marker.color.r = 1.0;
    corners_marker.color.g = 1.0;
    corners_marker.color.a = 1.0;
    corners_marker.scale.x = 0.5;
    corners_marker.scale.y = 0.5;
    corners_marker.scale.z = 0.5;

    auto iter2 = p_curr->landmarks_3d.begin();
    while (iter2 != p_curr->landmarks_3d.end()) {
      geometry_msgs::Point pt;
      pt.x = iter2->second[0];
      pt.y = iter2->second[1];
      pt.z = iter2->second[2];
      corners_marker.points.push_back(pt);
      ++iter2;
    }

    corners_viz_pub_.publish(corners_marker);
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

    corners_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/SimpleVo/corners_viz", 1);
  }

  void SimpleVo::LoadGateNominalInformation() {
    gate_corners_.clear();
    gate_corners_.push_back(std::vector<Eigen::Vector3d>());

    gate_corners_rel_.clear();
    gate_corners_rel_.push_back(std::vector<cv::Point3d>());

    constexpr std::size_t num_gates = 23;

    char buf[256];

    for (std::size_t i = 1; i <= num_gates; ++i) {
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
