#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "drake/lcm/drake_lcm.h"

namespace py = pybind11;

PYBIND11_PLUGIN(lcm) {

  py::module m("lcm", "Drake LCM bindings");
  using drake::lcm::DrakeLcm;
  using drake::lcm::DrakeLcmInterface;

  py::class_<DrakeLcmInterface> (m, "DrakeLcmInterface");

  py::class_<DrakeLcm, DrakeLcmInterface> (m, "DrakeLcm")
    .def(py::init<>());

  return m.ptr();
}
