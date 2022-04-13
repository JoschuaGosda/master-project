### Installation instruction
Installation of C++ dependencies can be a bit tricky. CMake is used for to compile the code. In order to do so create a './build' directory and run 'cmake ..'. This will use the CMakeLists.txt file to build a bunch of files. Then run 'make' within the build directory to build the program. Some notes:
1. make sure broccoli is installed locally to your pc. It is referred to in the cmake file as eat::broccoli within the 'target_link_libraries'
2. potentially modify the path for the 'include_directories' to where your Robotics Library installation lies
3. Have Eigen installed and find it with the path specified in 'FindEigen3.cmake'
