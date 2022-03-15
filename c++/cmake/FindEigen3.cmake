#
# This file is part of broccoli.
# Copyright (c) 2019 Chair of Applied Mechanics, Technical University of Munich
# https://www.amm.mw.tum.de/
#

# Find the external-project Eigen3 we are using with broccoli

find_path(EIGEN3_INCLUDE_DIRS Eigen/Eigen
        #PATHS "${CMAKE_BINARY_DIR}/eigen-src"
        PATHS "/opt/cppDeps/eigen-3.4.0"
        )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Eigen3 REQUIRED_VARS EIGEN3_INCLUDE_DIRS)