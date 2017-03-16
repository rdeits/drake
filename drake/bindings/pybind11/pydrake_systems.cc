#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

typedef RigidBodyTree<double> RigidBodyTree_d;
namespace py = pybind11;

PYBIND11_PLUGIN(systems) {

  py::module m("systems", "Drake systems");
  using drake::lcm::DrakeLcmInterface;
  using drake::systems::DrakeVisualizer;

  py::module::import("pydrake.lcm");

  py::class_<DrakeVisualizer> (m, "DrakeVisualizer")
    .def(py::init<const RigidBodyTree_d&, DrakeLcmInterface*, bool>(),
         py::arg("tree"), py::arg("lcm"), py::arg("enable_playback") = false)
    .def("PlaybackTrajectory", &DrakeVisualizer::PlaybackTrajectory)
    .def("set_publish_period", &DrakeVisualizer::set_publish_period);

  return m.ptr();
}
