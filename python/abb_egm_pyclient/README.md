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

### Build the shared library and put it into correct directory to import it in Python
```
cd c++/build
cmake ..
make
cp *.so ../../python/libs/
```


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
cd python/abb_egm_pyclient
pip install -e .
```

5. Install all other dependencies with `pip install -r requirements.txt`

6. Install the `python/data` folder as a module
```
cd python
pip install -e .
```
