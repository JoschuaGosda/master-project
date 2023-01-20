# Project description
A Novel Hybrid Force Motion Control Approach to Hot Wire Foam Cutting implemented to a ABB Yumi (IRB14000) robot. See a demonstration video here: [https://youtu.be/Q_Iuq7mRmcw](https://youtu.be/Q_Iuq7mRmcw)

# Setup 
## Kinematics implementation
C++ and C++ libraries are used to implement forward and inverse kinematics for the Yumi.

The Robotics Library is used to parse the robotic configuration from an .urdf file and compute the forward kinematics as well as the Jacobian. 

For inverse kinematics the broccoli library of the Chair of Applied Mechanics at TUM is used to implement a gradient based projection method which takes two additional constraints - a norm for maximal joint values and a norm for the distance of any singularities -  into account and projects them into the nullspace of the robot. Projecting these constraints into the nullspace helps to avoid maximal joint values and singularities without affecting the pose of the end effector.

## Interface to robot
Communication to the IRB14000 is performed via an UDP interface using Google Protobuffres. It is aimed to use an existing Python client and calling C++ code for the kinematics from the Python program. 

## Force sensing
In order to keep the wire tensioned during the foam cutting application sensor data is needed to setup a feedback loop. Therefore a one dimensional [load cell](https://www.electrokit.com/en/product/load-cell-1kg/) is going to be mounted to the end effector of Yumi. The signal of the full bridge that meassures the gauge of the cell is amplified by an HX711 AD-Converter and conntected to an Arduino Uno. The data is then going to be send to the controller that runs on the pc over a Serial Interface. 

## Hot Wire
The wire is going to be heated by appling a voltage that results in a current, depending on the resistance of the wire. The current is going to be controlled using a dc converter.


# Installation instructions
## C++ 
The project needs a few dependencies to be installed. These are broccoli, the robotics library as well as pybind11 in order to create bindings between Python and C++ code. To prevent any troubles while installing them in the next step make sure you have the following packages installed. Check or otherwise install them running `sudo apt install cmake python3-dev python3-distutils` in the command line.

Download eigen and put them into your `/opt/` folder.
### pybind11
There are several ways how to use pybind11. Number 1 was tested succesfully and is recommanded

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
Broccoli can be directly installed to the system (tested and works) or included as an external project. To install broccoli directly to the system clone the respective repo using git. Then cd in cloned brocolli repo and run the following commands to install brocolli to your system
```
mkdir build
cd build
cmake ..
make install
```
Broccoli and installation instruction can be found [here](https://gitlab.control.lth.se/gosda/broccoli-library).

### Robotics Library
To install the whole robotics library to your system run:
```
sudo apt-add-repository ppa:roblib/ppa
sudo apt-get update
sudo apt-get install librl librl-demos librl-examples
sudo apt-get install librl-dev
sudo apt-get install librl-extras librl-doc librl-dbg

```
Official installation instruction can be found [here](https://www.roboticslibrary.org/tutorials/install-ubuntu/).

## Python
Follow the steps to create a virtual environment and install the neccessary dependencies:

1. Create a directory for your virtual environment within the python folder
2. Create a virtual environment running `python3 -m venv ./myvenv`
3. Activate the virtual environment running `source myvenv/bin/activate`
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
