#ifndef _FITTING_HPP_
#define _FITTING_HPP_

#include <Eigen/Dense>
#include "Eigen/Spline/Spline.h"
#include "Eigen/Spline/SplineFitting.h"

#include <boost/function.hpp>

using namespace std;

namespace spline_planner {

  template<unsigned int D>
  class Spline3D {
  public:
    typedef Eigen::Spline3d SplineType;

    typedef boost::function<void(double, double, const Eigen::MatrixXd&)> SampleCallbackType;

    /*
     * points: input points for fitting spline
     */
    Spline3D(const Eigen::MatrixXd &points, const Eigen::MatrixXd &derivatives, const Eigen::VectorXi &index) {
      this->spline_ = Eigen::SplineFitting<Spline3D::SplineType>::InterpolateWithDerivatives(points, derivatives,
                                                                                             index, D);
    }

    /*
     * points: input points for fitting spline
     */
    Spline3D(const Eigen::MatrixXd &points) {
      Spline3D::SplineType::KnotVectorType chord_vector;
      Eigen::ChordLengths(points, chord_vector);

      this->spline_ = Eigen::SplineFitting<Spline3D::SplineType>::Interpolate(points, D, chord_vector);
    }

    void Sample(SampleCallbackType callback, double sample_ds = 0.1, double s_limit = 100.0, unsigned int degree = D, double dt = 0.01) {
      std::size_t i = 0;
      double t = 0.0;
      double s = 0.0;
      double last_s = 0.0;
      double last_deriv1_norm = 0;
      while ((t = i++ * dt) <= 1 && s <= s_limit) {
        const Eigen::MatrixXd& derivs = spline_.derivatives(t, degree);
        double deriv1_norm = static_cast<const Eigen::VectorXd&>(derivs.col(1)).norm();
        if (t > 1e-7) {
          s += 0.5 * (deriv1_norm + last_deriv1_norm) * dt;
        }
        last_deriv1_norm = deriv1_norm;

        if ((s - last_s) >= sample_ds || (i * dt) > 1.0 || s <= 1e-7) {
          last_s = s;
          callback(t, s, derivs);
        }
      }
    }

  private:
    SplineType spline_;

  };
}

#endif
