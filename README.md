<<<<<<< HEAD
# Project description
To come...

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

=======
# Kinematics implementation
Using the Robotics Library kinematic configuration of the robot can be specified using a xml or urdf file. At first the DH-parameters of an past project were used and implemented according to the tutorial of RL. RL uses the standard convention for specifing the dh parameter, namely:


- d: specifies the translation along the previous z axis
- theta: is the rotation around the previous z axis to align with the new x axis
- a: defines the translation along the new x axis
- alpha: gives the rotation around the new x axis to align with the new z axis

Unfortunately the parameters are for an older version of YuMi called Frida and do not match the TCP position and orientation when comparing against RobotStudio in a given joint configuration.
Fortunately there is another option to specifies the kinematics for a robot over a urdf file and there is already a model existing in the repo of OrebroUniversity. Their urdf-description file was then edited (which is not recommended but works) in order to specify the kinematic configuration for the left and right arm of YuMi. When comparing the resulting positions and orientations of the forward kinematics there is still an offset between the manual forward kinematics and RobotStudio. This is because the TCP-reference frame is not the same, as it can be seen in picture xy.
>>>>>>> f61f214bbe7cde1918af9bb232f3b67cab4d4a9c

# Installation instructions
## C++ 
The project needs a few dependencies to be installed. These are broccoli, the robotics library as well as pybind11 in order to create bindings between Python and C++ code. To prevent any troubles while installing them in the next step make sure you have the following packages installed. Check or otherwise install them running `sudo apt install cmake python3-dev python3-distutils` in the command line.

<<<<<<< HEAD
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
=======
Download the robotics library and eigen and put them into `/opt/`. Brocolli and pybind11 are directly installed to the system. Therefore clone the respective repo using git. 
### pybind11
Run the following commands to install pybind11 globally to your system
>>>>>>> f61f214bbe7cde1918af9bb232f3b67cab4d4a9c
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
<<<<<<< HEAD
Broccoli can be directly installed to the system (tested and works) or included as an external project. To install broccoli directly to the system clone the respective repo using git. Then cd in cloned brocolli repo and run the following commands to install brocolli to your system
=======
cd in cloned brocolli repo. Then run the commands to install brocolli to your system
>>>>>>> f61f214bbe7cde1918af9bb232f3b67cab4d4a9c
```
mkdir build
cd build
cmake ..
make install
```
<<<<<<< HEAD
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

# Troubles
Compiling the project with cmake runs without errors but when trying to import the python binding the following error appears:
`ImportError: /home/joschua/Coding/forceControl/master-project/c++/build/example.cpython-38-x86_64-linux-gnu.so: undefined symbol: _Z3gpmRN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES2_RNS0_IdLi7ELi1ELi0ELi7ELi1EEES4_S4_ddi`
The error can be demangled using an online demangler like [https://demangler.com](https://demangler.com)
The python binding code (the generated *.so file) can be imported by running `import example` within an interactive python session. An official example can be found [here](https://pybind11.readthedocs.io/en/stable/basics.html) 






=======
>>>>>>> f61f214bbe7cde1918af9bb232f3b67cab4d4a9c

