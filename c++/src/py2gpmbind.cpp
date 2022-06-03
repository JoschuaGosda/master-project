#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "yumi.hpp"
#include <rl/mdl/UrdfFactory.h>

namespace py = pybind11;



PYBIND11_MODULE(invKin, m) {
    m.doc() = "pybind11 binding to C++ function that computes IK based on GPM"; // optional module docstring

    py::class_<Yumi>(m, "Yumi")
       .def(py::init<std::string>())
       .def("set_jointValues", &Yumi::set_jointValues, py::arg("joint angles"), py::arg("joint velocities"))
       .def("set_desPoseVel", &Yumi::set_desPoseVel, py::arg("desired pose"), py::arg("desired velocities"))
       .def("process", &Yumi::process)
       .def("get_newJointValues", &Yumi::get_newJointValues, py::return_value_policy::copy)
       .def("get_pose", &Yumi::get_pose, py::return_value_policy::copy)
       .def("printPose", &Yumi::print_pose)
       .def("set_kp", &Yumi::set_kp)
       .def("set_operationPoint", &Yumi::set_operationPoint)
       .def("set_hybridControl", &Yumi::set_hybridControl)
       .def("set_transitionTime", &Yumi::set_transitionTime)
       .def("set_force", &Yumi::set_force);
	}