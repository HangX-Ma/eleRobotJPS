# Matplotlib Usage

```C++
#include "matplotlibcpp.h"
```
[lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp) is used in this `toppra`. This is a wrapper file for connivent `Matplotlib` in C++. Before your usage, you need to install the `python-matplotlib` at first.

If you are on Linux, you might prefer to use your package manager. `matplotlib` is packaged for almost every major Linux distribution.

```shell
  Debian / Ubuntu : sudo apt-get install python-matplotlib python-numpy python2.7-dev
```
The example can be seen from [lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp).

# TOPP-RA

[toppra](https://github.com/hungpham2511/toppra) is a module for trajectory planning. In this github address, you need to enter the `cpp` file. If you want to use the `toppra`, you need to install some prerequisite modules.

### pinocchio
pinocchio is used in toppra to calculate kinematical quantities such as end-effector pose jacobians, required for task-space velocity consraint.

Install pinocchio bindings (see below) and example from robotpkg, then setup environment variables before building

```shell
# Building
export LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH}
export CMAKE_PREFIX_PATH=/opt/openrobots
```
To run the tests
```shell
# Running
export ROS_PACKAGE_PATH=/opt/openrobots/share
export PYTHONPATH=/opt/openrobots/lib/python3.6/site-packages:$PYTHONPATH
# or any python version that you install
```
### optional dependencies:
Install optional dependencies:
- GLPK: `sudo apt install libglpk-dev`
- qpOASES: `sudo apt install robotpkg-qpoases` (follow http://robotpkg.openrobots.org/debian.html for robotpkg)
- pinocchio: `sudo apt install robotpkg-pinocchio` (follow http://robotpkg.openrobots.org/debian.html for robotpkg)

### pybind11
We use `pybind11` to provide the bindings. To build the bindings you need to first install `pybind11` version `2.5.0`.

```shell
cd ~/ && git clone https://github.com/pybind/pybind11
cd pybind11 && git checkout v2.5.0
mkdir build && cd build && cmake -DPYBIND11_TEST=false .. && sudo make install
```


Build toppra with Python bindings and all optional dependencies:

```shell
cmake -DBUILD_WITH_PINOCCHIO=ON -DBUILD_WITH_qpOASES=ON -DBUILD_WITH_GLPK=ON -DPYTHON_BINDINGS=ON -DPYBIND11_PYTHON_VERSION=3.7 ..
make
```

### Building doxygen doc
```shell
# Run cmake normally
make doc
# The documentation is available at doc/doc/html/index.html
```

### Using TOPPRA in CMake-based project

```cmake
# The following line defines cmake target toppra::toppra
find_package(toppra)
...
target_link_library(foo PUBLIC toppra::toppra)
# foo is the target name. PUBLIC toopra::toppra need to put in front.
```

## robotpkg
[robotpkg](http://robotpkg.openrobots.org/debian.html) installation 

tell apt where to find the package summary and other administrative files, for the desired architecture, for instance with the following command:

``` shell
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF 
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg 
EOF
```

register the robotpkg authentication key like so:
```shell
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```

You need to run at least once apt-get update to fetch the package descriptions:
```shell
sudo apt-get update
```

### Installing packages

`sudo apt-get install robotpkg-<PACKAGE>`