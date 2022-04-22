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

Download the robotics library and eigen and put them into `/opt/`.
### pybind11
There are several ways how to use pybind11. Number 1 was tested succesfully

1. Inlcude pybind11 as submodule
Within the `c++` folder run
```
git submodule add -b stable https://github.com/pybind/pybind11 deps/pybind11
git submodule update --init
```

2. Run the following commands to install pybind11 globally to your system
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
Brocolli can be directly installed to the system. Therefore clone the respective repo using git. Then cd in cloned brocolli repo and run the commands to install brocolli to your system
```
mkdir build
cd build
cmake ..
make install
```
## Python
 To keep things clean it is best to install them within a virtual environment. Follow the steps to create a virtual environment and install the neccessary dependencies:

Create a directory for your virtual environment within the python folder
Create a virtual environment running `python3 -m venv /path/to/venv/directory`
Activate the virtual environment running `source /path/to/venv/directory/bin/activate`
Install the dependencies for the abb-egm-client following the instructions in the respective README file
Install all other dependencies with `pip install -r requirements.txt`
1. Create a directory for your virtual environment within the python folder
2. Create a virtual environment running `python3 -m venv /path/to/venv/directory`
3. Activate the virtual environment running `source /path/to/venv/directory/bin/activate`
4. Install the dependencies for the abb-egm-client:

- Install protoc from protobuf. There are prebuilt binaries available from
GitHub if protoc is not available in your package manager.
- Find egm.proto in either %LOCALAPPDATA%\ABB\RobotWare\RobotWare_6.XXXXX\utility\Template\EGM\egm.proto or on the robot.


Run protoc to generate protobuf classes for python. Substitute `$SRC_DIR` for the location of `egm.proto`

`protoc --python_out=abb_egm_pyclient $SRC_DIR/egm.proto`

Install this package in your environment of choice.

```
cd abb_egm_pyclient
pip install -e .
```

5. Install all other dependencies with `pip install -r requirements.txt`



