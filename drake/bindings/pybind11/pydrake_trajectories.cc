#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace py = pybind11;

PYBIND11_PLUGIN(trajectories) {

  py::module m("trajectories", "Piecewise polynomial trajectories");

  py::class_<PiecewisePolynomial<double>>(m, "PiecewisePolynomial")
    .def(py::init<>())
    .def_static("ZeroOrderHold", [](
      const std::vector<double>& breaks,
      const std::vector<Eigen::MatrixXd>& knots) {
        return PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);
      })
    .def_static("FirstOrderHold", [](
      const std::vector<double>& breaks,
      const std::vector<Eigen::MatrixXd>& knots) {
        return PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
      })
    .def_static("Pchip", [](
      const std::vector<double>& breaks,
      const std::vector<Eigen::MatrixXd>& knots,
      bool zero_end_point_derivatives) {
        return PiecewisePolynomial<double>::Pchip(breaks, knots,
                                                  zero_end_point_derivatives);
      }, py::arg("breaks"), py::arg("knots"),
      py::arg("zero_end_point_derivatives") = false);

  return m.ptr();
}
