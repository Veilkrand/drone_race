#ifndef _SIMPLE_VO_G2O_TYPES_H_
#define _SIMPLE_VO_G2O_TYPES_H_

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace simple_vo {
  class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out) const {}

  private:
    Eigen::Matrix3d k_;
    Eigen::Vector3d point_;

  public:
    void set_k(const Eigen::Matrix3d& k) { k_ = k; }
    void set_point(const Eigen::Vector3d& point) { point_ = point; }
    Eigen::Matrix3d& k() { return k_; }
    Eigen::Vector3d& point() { return point_; }
  };
}

#endif
