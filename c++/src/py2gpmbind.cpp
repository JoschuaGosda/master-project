#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "gpm.hpp"
#include "loadKinematicModel.hpp"
#include "yumi.hpp"
#include <rl/mdl/UrdfFactory.h>

namespace py = pybind11;



PYBIND11_MODULE(invKin, m) {
    m.doc() = "pybind11 binding to C++ function that computes IK based on GPM"; // optional module docstring

    //m.def("gpm", &gpm, "A function to compute the invserse kinematics for a 7 DOF serial manipulator on vecocity level with comfort pose and manipulabilty gradient",
    //py::arg("desired pose"),py::arg("desired Velocities"), py::arg("joint angles"), py::arg("joint velocities"), py::arg("left arm -> 1, right arm -> 0"), py::return_value_policy::copy);
	
    py::class_<Yumi>(m, "Yumi")
       .def(py::init<std::string>())
       .def("set_jointValues", &Yumi::set_jointValues, py::arg("joint angles"), py::arg("joint velocities"))
       .def("printPose", &Yumi::print_pose); 
    // load model and return ptr for future function calls
    // https://pybind11.readthedocs.io/en/stable/advanced/functions.html#return-value-policies

    /* py::class_<Arm>(m, "Arm")
       .def(py::init<std::string>())
       .def("get_pointer2arm", &Arm::get_pointer2arm); */
	}