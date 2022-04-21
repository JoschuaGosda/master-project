# Kinematics implementation
Using the Robotics Library kinematic configuration of the robot can be specified using a xml or urdf file. At first the DH-parameters of an past project were used and implemented according to the tutorial of RL. RL uses the standard convention for specifing the dh parameter, namely:


- d: specifies the translation along the previous z axis
- theta: is the rotation around the previous z axis to align with the new x axis
- a: defines the translation along the new x axis
- alpha: gives the rotation around the new x axis to align with the new z axis

Unfortunately the parameters are for an older version of YuMi called Frida and do not match the TCP position and orientation when comparing against RobotStudio in a given joint configuration.
Fortunately there is another option to specifies the kinematics for a robot over a urdf file and there is already a model existing in the repo of OrebroUniversity. Their urdf-description file was then edited (which is not recommended but works) in order to specify the kinematic configuration for the left and right arm of YuMi. When comparing the resulting positions and orientations of the forward kinematics there is still an offset between the manual forward kinematics and RobotStudio. This is because the TCP-reference frame is not the same, as it can be seen in picture xy.

# Installation instructions
## C++ 
The project needs a few dependencies to be installed. These are broccoli, the robotics library as well as pybind11 in order to create bindings between Python and C++ code. To prevent any troubles while installing them in the next step make sure you have the following packages installed. Check or otherwise install them running `sudo apt install cmake python3-dev python3-distutils` in the command line.

Download the robotics library and eigen and put them into `/opt/`. Brocolli and pybind11 are directly installed to the system. Therefore clone the respective repo using git. 
### pybind11
Run the following commands to install pybind11 globally to your system
```
# Classic CMake
cd pybind11
mkdir build
cd build
cmake ..
make install

# CMake 3.15+
cd pybind11
cmake -S . -B build
cmake --build build -j 2  # Build on 2 cores
cmake --install build
```
Installation instruction for pybind11 can be also found [here](https://pybind11.readthedocs.io/en/stable/compiling.html#building-with-cmake)

### broccoli
cd in cloned brocolli repo. Then run the commands to install brocolli to your system
```
mkdir build
cd build
cmake ..
make install
```

