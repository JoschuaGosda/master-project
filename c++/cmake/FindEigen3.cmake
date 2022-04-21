# Find external-project Eigen3 that is used within broccoli

find_path(EIGEN3_INCLUDE_DIRS Eigen/Eigen
        #PATHS "${CMAKE_BINARY_DIR}/eigen-src"
        PATHS "/opt/eigen-3.4.0"
        )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIRS)