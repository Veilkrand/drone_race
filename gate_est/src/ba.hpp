#if !defined(_GATE_EST_BA_HPP_)
#define _GATE_EST_BA_HPP_

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace gate_est {

  class CostFunctionReProjection {
    public:
    inline CostFunctionReProjection(
      const Eigen::Vector3d& pt,
      const Eigen::MatrixXd& t,
      const Eigen::Matrix3d& cm,
      const Eigen::Vector2d& uv)
      : fx_(cm(0, 0)), fy_(cm(1, 1)), cx_(cm(0, 2)), cy_(cm(1, 2)),
        u_(uv[0]), v_(uv[1]),
        t_(t),
        pt_x_(pt[0]), pt_y_(pt[1]), pt_z_(pt[2]) { }

    template <typename T>
    inline bool operator() (const T* const point, T* residual) const {
      auto x = point[0] + T(pt_x_);
      auto y = point[1] + T(pt_y_);
      auto z = point[2] + T(pt_z_);

      auto px = T(t_(0, 0)) * x + T(t_(0, 1)) * y + T(t_(0, 2)) * z  + T(t_(0, 3));
      auto py = T(t_(1, 0)) * x + T(t_(1, 1)) * y + T(t_(1, 2)) * z  + T(t_(1, 3));
      auto pz = T(t_(2, 0)) * x + T(t_(2, 1)) * y + T(t_(2, 2)) * z  + T(t_(2, 3));

      auto u = T(fx_) * px / pz + T(cx_);
      auto v = T(fy_) * py / pz + T(cy_);

      residual[0] = T(u_) - u;
      residual[1] = T(v_) - v;

      return true;
    }

    private:
      const double fx_;
      const double fy_;
      const double cx_;
      const double cy_;
      const double u_;
      const double v_;
      const double pt_x_;
      const double pt_y_;
      const double pt_z_;
      const Eigen::MatrixXd t_;
  };
  
} // gate_est


#endif // _GATE_EST_BA_HPP_
