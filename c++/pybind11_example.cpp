#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "gpm.hpp"

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}


PYBIND11_MODULE(invKin, m) {
    m.doc() = "pybind11 binding to C++ function that computes IK based on GPM"; // optional module docstring
    m.def("gpm", &gpm, "A function to compute the invserse kinematics for a 7 DOF serial manipulator on vecocity level with comfort pose and manipulabilty gradient",
    py::arg("desired pose"),py::arg("desired Velocities"), py::arg("joint angles"), py::arg("joint velocities"), py::arg("left arm -> 1, right arm -> 0"), py::return_value_policy::copy);
	
	}