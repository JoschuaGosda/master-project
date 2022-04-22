#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;


int add(int i, int j) {
    return i + j;
}


PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 binding to C++ function that computes IK based on GPM"; // optional module docstring
    m.def("add", add, "A function that takes the current joint values and the desired task space values for the next step and returns the necessary joint angles, respectively", py::return_value_policy::copy);
	
	}