
find_path(RL_INCLUDE_DIRS 
        PATHS "/opt/rl-0.7.0"
        )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIRS)