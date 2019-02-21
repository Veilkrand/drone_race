#include <boost/python.hpp>
#include <boost/assert.hpp>
#include <vector>
#include <Eigen/Dense>

#include "eigen_spline.hpp"

using namespace spline_planner;

namespace py = boost::python;

py::list ColumnMajorMatrixToRowMajorPython2DList(const Eigen::MatrixXd& mat) {
  py::list result;
  const size_t cols = mat.cols();
  const size_t rows = mat.rows();
  for (size_t i = 0; i < cols; ++i) {
    py::list vec;
    for (size_t j = 0; j < rows; ++j) {
      vec.append(mat(j, i));
    }
    result.append(vec);
  }
  return result;
}

Eigen::MatrixXd RowMajorPython2DListToColumnMajorMatrix(py::list list) {
  const size_t rows = py::len(list);
  const size_t cols = py::len(list[0]);
  Eigen::MatrixXd mat(rows, cols);
  for (size_t r = 0; r < rows; ++r) {
    py::list row = static_cast<py::list>(list[r]);
    for (size_t c = 0; c < cols; ++c) {
      mat(c, r) = py::extract<double>(row[c]);
    }
  }
  return mat;
}

struct SplineSamplingCallback {

  void operator()(double t, double s, const Eigen::MatrixXd& derivatives) {
    py::list point;
    point.append(t);
    point.append(s);
    point.append(ColumnMajorMatrixToRowMajorPython2DList(derivatives));
    result.append(point);
  }

  py::list result;

};

typedef spline_planner::Spline3D<3> CubicSpline3D;

struct CubicSpline3DWrapper {
  
  CubicSpline3DWrapper(py::list points)
    : spline_(nullptr) {
    const int num_pts = py::len(points);
    Eigen::MatrixXd mat(3, num_pts);
    for (size_t i = 0; i < num_pts; ++i) {
      mat(0, i) = py::extract<double>(points[i][0]);
      mat(1, i) = py::extract<double>(points[i][1]);
      mat(2, i) = py::extract<double>(points[i][2]);
    }
    spline_ = new CubicSpline3D(mat);
  }

  CubicSpline3DWrapper(py::list points, py::list derivatives, py::list index)
    : spline_(nullptr) {
    const int num_pts = py::len(points);
    Eigen::MatrixXd mat(3, num_pts);
    for (size_t i = 0; i < num_pts; ++i) {
      mat(0, i) = py::extract<double>(points[i][0]);
      mat(1, i) = py::extract<double>(points[i][1]);
      mat(2, i) = py::extract<double>(points[i][2]);
    }
    
    const int num_derivs = py::len(derivatives);
    Eigen::MatrixXd mat_derivs(3, num_derivs);
    for (size_t i = 0; i < num_derivs; ++i) {
      mat_derivs(0, i) = py::extract<double>(derivatives[i][0]);
      mat_derivs(1, i) = py::extract<double>(derivatives[i][1]);
      mat_derivs(2, i) = py::extract<double>(derivatives[i][2]);
    }

    Eigen::VectorXi vec_index(num_derivs);
    for (size_t i = 0; i < num_derivs; ++i) {
      vec_index(i) = py::extract<int>(index[i]);
    }
    
    spline_ = new CubicSpline3D(mat, mat_derivs, vec_index);
  }

  py::list SampleAndCollect0() {
    return SampleAndCollect();
  }

  py::list SampleAndCollect1(double sample_ds) {
    return SampleAndCollect(sample_ds);
  }

  py::list SampleAndCollect2(double sample_ds, double s_limit) {
    return SampleAndCollect(sample_ds, s_limit);
  }

  py::list SampleAndCollect(double sample_ds = 0.1, double s_limit=100.0, double dt = 0.01) {
    SplineSamplingCallback callback;
    spline_->Sample(callback, sample_ds, s_limit, 2, dt);
    return callback.result;
  }

  ~CubicSpline3DWrapper() {
    if (spline_) {
      delete spline_;
    }
  }

  CubicSpline3D* spline_;
};

BOOST_PYTHON_MODULE(eigen_spline) {

  py::class_<CubicSpline3DWrapper>("CubicSpline3DWrapper", py::init<py::list>())
    .def(py::init<py::list, py::list, py::list>())
    .def("sampleAndCollect", &CubicSpline3DWrapper::SampleAndCollect0)
    .def("sampleAndCollect", &CubicSpline3DWrapper::SampleAndCollect1)
    .def("sampleAndCollect", &CubicSpline3DWrapper::SampleAndCollect2)
    .def("sampleAndCollect", &CubicSpline3DWrapper::SampleAndCollect);
    
}
