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

Create a directory for your virtual environment within the python folder
Create a virtual environment running `python3 -m venv /path/to/venv/directory`
Activate the virtual environment running `source /path/to/venv/directory/bin/activate`
Install the dependencies for the abb-egm-client following the instructions in the respective README file
Install all other dependencies with `pip install -r requirements.txt`
