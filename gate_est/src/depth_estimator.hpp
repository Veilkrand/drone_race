#ifndef _DEPTH_ESTIMATOR_H_
#define _DEPTH_ESTIMATOR_H_

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <ceres/ceres.h>

namespace depth_estimator {

  class CostFunction {
  public:
    explicit inline CostFunction(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                 int idx1, int idx2,
                                 double target_dist)
      : x1_(p1[0]), y1_(p1[1]),
        x2_(p2[0]), y2_(p2[1]),
        idx1_(idx1), idx2_(idx2),
        dist_(target_dist) {}

    template <typename T>
    inline bool operator() (const T* const d, T* residual) const {
      residual[0] = T(dist_) - ceres::sqrt(ceres::pow(d[idx1_] * T(x1_) - d[idx2_] * T(x2_), 2) +
                                           ceres::pow(d[idx1_] * T(y1_) - d[idx2_] * T(y2_), 2) +
                                           ceres::pow(d[idx1_]          - d[idx2_],          2));
      return true;
    }

  private:
    const double x1_;
    const double y1_;
    const double x2_;
    const double y2_;
    const int idx1_;
    const int idx2_;
    const double dist_;
  };

  template <int N>
  class DepthEstimator {
  public:
    inline DepthEstimator(const std::vector<Eigen::Vector2d>& points_2d,
                          const std::vector<Eigen::Vector3d>& points_3d) 
      : points_2d_(points_2d), points_3d_(points_3d) {}

    inline Eigen::VectorXd Estimate() {
      std::vector<double> depths(N, 1.0);

      ceres::Problem problem;

      for (int i = N - 1; i >= 1; --i) {
        for (int j = i - 1; j >= 0; --j) {
          problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CostFunction, 1, N>(
              new CostFunction(points_2d_[i], points_2d_[j],
                               i, j,
                               (points_3d_[i] - points_3d_[j]).norm())),
            nullptr, depths.data());
        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;
      options.parameter_tolerance = 0.01;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      Eigen::VectorXd result(N);
      for (int i = 0; i < N; ++i) {
        result[i] = depths[i];
      }
      return result;
    }

  private:
    const std::vector<Eigen::Vector2d>& points_2d_;
    const std::vector<Eigen::Vector3d>& points_3d_;
  };

}

#endif
