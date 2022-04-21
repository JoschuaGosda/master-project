# Installation instructions
## C++ 
The project needs a few dependencies to be installed. These are broccoli, the robotics library as well as pybind11 in order to create bindings between Python and C++ code. To prevent any troubles while installing them in the next step make sure you have the following packages installed. Check or otherwise install them running `sudo apt install cmake python3-dev python3-distutils` in the command line.

Download the robotics library and put it into `/opt/`. Brocolli and pybind11 are directly installed to the system. Therefore clone the respective repo using git. 
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

## Python
 To keep things clean it is best to install them within a virtual environment. Follow the steps to create a virtual environment and install the neccessary dependencies:

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
