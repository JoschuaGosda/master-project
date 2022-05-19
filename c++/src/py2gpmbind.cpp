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
       .def("get_newJointValues", &Yumi::get_newJointValues)
       .def("get_newPose", &Yumi::get_newPose)
       .def("printPose", &Yumi::print_pose); 
	}