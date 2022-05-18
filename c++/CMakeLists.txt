cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(invKin VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14) # broccoli needs c++ standard 14 or above
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(./include)
#link_directories(${MyProject_SOURCE_DIR}/src)

# find all the packages that are installed globally to the os
find_package (Eigen3 REQUIRED NO_MODULE)
find_package (broccoli 3.0.0 COMPONENTS eigen REQUIRED)
find_package(RL COMPONENTS MDL REQUIRED)

# dependencies for gpm function
add_library(my_funcs SHARED src/gpm.cpp src/loadKinematicModel.cpp src/pointer_example.cpp)
target_link_libraries (
    my_funcs 
    ${RL_LIBRARIES}
    eat::broccoli
    Eigen3::Eigen
)

# dependencies for pybind11
add_subdirectory(deps/pybind11) # include pybind11 as a submodule
pybind11_add_module(invKin src/py2gpmbind.cpp)
target_link_libraries(
    invKin PUBLIC 
    my_funcs
    ${RL_LIBRARIES}
    eat::broccoli
)

add_executable(myMain src/main.cpp src/gpm.cpp src/loadKinematicModel.cpp src/pointer_example.cpp)
target_link_libraries(
    myMain
    eat::broccoli
    ${RL_LIBRARIES}
)