#include "gate_est_node.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/Marker.h>

#include <cstring>
#include <string>
#include <algorithm>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "depth_estimator.hpp"

#include "ba.hpp"

namespace gate_est
{

cv::Mat TransformToCvProjection(const geometry_msgs::Transform &transform)
{
  Eigen::Quaterniond quat(transform.rotation.w,
                          transform.rotation.x,
                          transform.rotation.y,
                          transform.rotation.z);
  Eigen::Matrix3d rot = quat.toRotationMatrix();

  cv::Mat proj = (cv::Mat_<double>(3, 4) << rot(0, 0), rot(0, 1), rot(0, 2), transform.translation.x,
                  rot(1, 0), rot(1, 1), rot(1, 2), transform.translation.y,
                  rot(2, 0), rot(2, 1), rot(2, 2), transform.translation.z);
  return proj;
}

cv::Point2d GateEst::Pixel2Camera(double x, double y)
{
  const double cx = k_.at<double>(0, 2);
  const double cy = k_.at<double>(1, 2);
  const double fx = k_.at<double>(0, 0);
  const double fy = k_.at<double>(1, 1);
  return cv::Point2d((x - cx) / fx, (y - cy) / fy);
}

void GateEst::BackProject(double x, double y, double depth, Eigen::Vector3d &out)
{
  const double cx = k_.at<double>(0, 2);
  const double cy = k_.at<double>(1, 2);
  const double fx = k_.at<double>(0, 0);
  const double fy = k_.at<double>(1, 1);
  out << (x - cx) * depth / fx,
      (y - cy) * depth / fy,
      depth;
}

void GateEst::Pixel2Camera(double x, double y, Eigen::Vector2d &out)
{
  const double cx = k_.at<double>(0, 2);
  const double cy = k_.at<double>(1, 2);
  const double fx = k_.at<double>(0, 0);
  const double fy = k_.at<double>(1, 1);
  out << (x - cx) / fx,
      (y - cy) / fy;
}

void GateEst::Pixel2Camera(double x, double y, cv::Point2d &out)
{
  const double cx = k_.at<double>(0, 2);
  const double cy = k_.at<double>(1, 2);
  const double fx = k_.at<double>(0, 0);
  const double fy = k_.at<double>(1, 1);
  out.x = (x - cx) / fx;
  out.y = (y - cy) / fy;
}

void GateEst::OdometryCallback(const nav_msgs::OdometryConstPtr& ptr) {
/*   const Eigen::MatrixXd& corners_cam = T_c_w.matrix() * gate_corners_est_;
  Eigen::MatrixXd corners_projected = cm_ * corners_cam.topRows(3);
  corners_projected.array().rowwise() /= corners_cam.row(2).array();

  ROS_INFO_STREAM(corners_projected.col(9 * 4 + 0));
  ROS_INFO_STREAM(corners_projected.col(9 * 4 + 1));
  ROS_INFO_STREAM(corners_projected.col(9 * 4 + 2));
  ROS_INFO_STREAM(corners_projected.col(9 * 4 + 3));
 */
  // 2. optimize gate corner position via bundle adjustments.

  geometry_msgs::TransformStamped transform;
  try {
    transform = buffer_.lookupTransform("camera", "map", ptr->header.stamp, ros::Duration(1.0/20));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Unable to get transform. Skipped.");
    return;
  }

  const Eigen::MatrixXd& T_c_w = tf2::transformToEigen(transform.transform).matrix();

  ceres::Problem problem;

  for (auto iter = p_ref_->landmarks_2d.begin(); iter != p_ref_->landmarks_2d.end(); ++iter)
  {
    std::size_t key = iter->first;
    std::size_t gate_idx = GetGateIndex(key) - 1;
    std::size_t corner_idx = GetCornerIndex(key) - 1;
    const Eigen::Vector2d& uv = iter->second;
    const Eigen::Vector3d& pt = gate_corners_[gate_idx][corner_idx];
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctionReProjection, 2, 3>(
        new CostFunctionReProjection(pt, T_c_w, cm_, uv)),
        nullptr, &gate_offsets_[gate_idx][0]);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.parameter_tolerance = 1e-5;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  visualization_msgs::Marker corners_marker;
  corners_marker.header.stamp = ros::Time::now();
  corners_marker.header.frame_id = "map";
  corners_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  corners_marker.action = visualization_msgs::Marker::ADD;
  corners_marker.id = 1;
  corners_marker.color.r = 1.0;
  corners_marker.color.g = 1.0;
  corners_marker.color.a = 1.0;
  corners_marker.scale.x = 0.5;
  corners_marker.scale.y = 0.5;
  corners_marker.scale.z = 0.5;

  ROS_INFO_STREAM("Gate offsets:");
  for (std::size_t i = 0; i < NUM_GATES; ++i) {
    ROS_INFO_STREAM("Gate "<<i+1<<": "<<gate_offsets_[i][0]<<", "<<gate_offsets_[i][1]<<", "<<gate_offsets_[i][2]);
    for (int j = 0; j < 4; ++j) {
      geometry_msgs::Point pt;
      pt.x = gate_corners_[i][j][0] + gate_offsets_[i][0];
      pt.y = gate_corners_[i][j][1] + gate_offsets_[i][1];
      pt.z = gate_corners_[i][j][2] + gate_offsets_[i][2];
      corners_marker.points.push_back(pt);
      std_msgs::ColorRGBA c;
      c.r = 1.0;
      c.g = 1.0;
      c.a = 1.0;
      corners_marker.colors.push_back(c);

      geometry_msgs::Point pt_n;
      pt_n.x = gate_corners_[i][j][0];
      pt_n.y = gate_corners_[i][j][1];
      pt_n.z = gate_corners_[i][j][2];
      corners_marker.points.push_back(pt_n);
      std_msgs::ColorRGBA c_n;
      c_n.b = 1.0;
      c_n.a = 1.0;
      corners_marker.colors.push_back(c_n);
    }
  }
  corners_viz_pub_.publish(corners_marker);
}

void GateEst::IRMarkerArrayCallback(const flightgoggles::IRMarkerArrayConstPtr& ptr) {
  // store detected irmarker information for later use
  auto frame = Frame::MakePtr();
  for (auto iter = ptr->markers.begin(); iter != ptr->markers.end(); ++iter)
  {
    std::size_t gate_idx = std::atoi(iter->landmarkID.data.substr(4).c_str());
    std::size_t idx = std::atoi(iter->markerID.data.c_str());

    StoreLandmark(frame->landmarks_2d, gate_idx, idx, Eigen::Vector2d(iter->x, iter->y));
  }
  p_ref_ = frame;
}

void GateEst::Init()
{
  // read in course definition and initial pose, then establish a
  // reference frame base on these information
  LoadGateNominalInformation();
  LoadInitialPose();
  reference_gate_idx_ = 10;

  camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/uav/camera/left/camera_info", 1, &GateEst::CameraInfoCallback, this);
  ir_beacons_sub_ = nh_.subscribe<flightgoggles::IRMarkerArray>("/uav/camera/left/ir_beacons", 1, &GateEst::IRMarkerArrayCallback, this);

  odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 1, &GateEst::OdometryCallback, this);

  corners_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/GateEst/corners_viz", 1);
}

void GateEst::LoadGateNominalInformation()
{
  gate_corners_.clear();

  char buf[256];

  for (std::size_t i = 1; i <= NUM_GATES; ++i)
  {
    std::sprintf(buf, "/uav/Gate%zu/nominal_location", i);
    XmlRpc::XmlRpcValue corners;
    nh_.getParam(buf, corners);
    std::vector<Eigen::Vector3d> cs;
    std::vector<cv::Point3d> cs_rel;
    for (std::size_t j = 0; j < 4; ++j)
    {
      cs.push_back(Eigen::Vector3d(corners[j][0], corners[j][1], corners[j][2]));
      if (j == 0)
      {
        // 0, 0, 0 for the first corner
        cs_rel.push_back(cv::Point3d(0, 0, 0));
      }
      else
      {
        // position relative to the first corner for the others
        const Eigen::Vector3d v = cs[j] - cs[0];
        cs_rel.push_back(cv::Point3d(v[0], v[1], v[2]));
      }
    }

    gate_corners_.push_back(cs);
  }
  ROS_INFO("Nominal gates information loaded.");
}

void GateEst::LoadInitialPose()
{
  std::vector<double> pose;
  nh_.getParam("/uav/flightgoggles_uav_dynamics/init_pose", pose);
  initial_position_ << pose[0], pose[1], pose[2];
  initial_orientation_ = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
}

void GateEst::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ptr)
{
  k_ = (cv::Mat_<double>(3, 3)
            << ptr->K[0],
        ptr->K[1], ptr->K[2],
        ptr->K[3], ptr->K[4], ptr->K[5],
        ptr->K[6], ptr->K[7], ptr->K[8]);

  cm_ << ptr->K[0],ptr->K[1], ptr->K[2],
         ptr->K[3], ptr->K[4], ptr->K[5],
         ptr->K[6], ptr->K[7], ptr->K[8];
}

} // namespace gate_est

XROS_RUNNABLE_NODE_MAIN(gate_est::GateEst)
