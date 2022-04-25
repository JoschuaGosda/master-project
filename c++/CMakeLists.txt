cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(invKin VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# include header files for rt and eigen
include_directories(/opt/eigen-3.4.0 /usr/include/python3.8) 

find_package (broccoli 3.0.0 COMPONENTS eigen REQUIRED)
find_package(RL COMPONENTS MDL REQUIRED)
# find_package(pybind11 CONFIG)

#link_directories(./)

# Build the C++ program
add_executable(myProgram gpm.cpp main.cpp gpm.hpp)
target_link_libraries (
    myProgram 
    ${RL_LIBRARIES}
    eat::broccoli
)

# Build the .so file to import in Python
#add_library(my_gpm SHARED gpm.cpp pybind11_example.cpp)
add_subdirectory(deps/pybind11)
pybind11_add_module(example pybind11_example.cpp)
#target_link_libraries(
#    example PUBLIC my_gpm
#    ${RL_LIBRARIES}
#    eat::broccoli
#)