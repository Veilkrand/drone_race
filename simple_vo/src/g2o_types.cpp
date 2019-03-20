#include "g2o_types.hpp"
#include "common.hpp"

namespace {
  Eigen::Vector2d ProjectPoint(const Eigen::Matrix3d& k, const Eigen::Vector3d& point) {
    return Eigen::Vector2d(
      point[0] * k(0, 0) / point[2] + k(0, 2),
      point[1] * k(1, 1) / point[2] + k(1, 2)
    );
  }
}

namespace simple_vo {

  void EdgeProjectXYZ2UVPoseOnly::computeError() {
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    _error = _measurement - ProjectPoint(k_, pose->estimate().map(point_));
  }

  void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    g2o::SE3Quat T(pose->estimate());
    const Eigen::Vector3d& pt = T.map(point_);
    double x = pt[0];
    double y = pt[1];
    double z = pt[2];
    double fx = k_(0, 0);
    double fy = k_(1, 1);
    double z_2 = z * z;

    _jacobianOplusXi(0, 0) = x * y / z_2 * fx;
    _jacobianOplusXi(0, 1) = -(1 + (x*x / z_2)) * fx;
    _jacobianOplusXi(0, 2) = y / z * fx;
    _jacobianOplusXi(0, 3) = -1. / z * fx;
    _jacobianOplusXi(0, 4) = 0;
    _jacobianOplusXi(0, 5) = x / z_2 * fx;

    _jacobianOplusXi(1, 0) = (1 + y*y / z_2) * fy;
    _jacobianOplusXi(1, 1) = -x*y / z_2  * fy;
    _jacobianOplusXi(1, 2) = -x/z * fy;
    _jacobianOplusXi(1, 3) = 0;
    _jacobianOplusXi(1, 4) = -1./z * fy;
    _jacobianOplusXi(1, 5) = y/z_2 * fy;
  }

}