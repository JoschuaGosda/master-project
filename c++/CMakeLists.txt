cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(invKin VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# include header files for rt, could not be found following the tutorial
include_directories(/usr/include/rl-0.7.0)

find_package (broccoli 3.0.0 COMPONENTS eigen REQUIRED)
find_package(RL COMPONENTS MDL REQUIRED)

add_executable(myProgram main.cpp)

target_link_libraries (
    myProgram 
    ${RL_LIBRARIES}
    eat::broccoli
)





