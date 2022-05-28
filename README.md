# `ZJUER:` :cn: Elephant Manipulator JPS Path Planning Manual

![ubuntu18.04](https://img.shields.io/badge/platform-ubuntu18.04-green) ![ros](https://img.shields.io/badge/ros-melodic-green) ![release date](https://img.shields.io/badge/release%20date-Aprial%202022-orange) ![build](https://img.shields.io/badge/build-passing-blue?logo=github) 

This is a repository that guides you to use the `JPS` path planning module, you can get the original files from [HangX-Ma](https://github.com/HangX-Ma)/[ROKAE-deficient-dof-manipulator](https://github.com/HangX-Ma/ROKAE-deficient-dof-manipulator.git).

This project runs on `ubuntu18.04` machine. **Elephant** manipulator `Panda3` `6-DOF` manipulator model is used. If you want to run this project on your machine, you need to install the necessary packages.

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [`ZJUER:` :cn: Elephant Manipulator JPS Path Planning Manual](#zjuer-cn-elephant-manipulator-jps-path-planning-manual)
  - [_TODO LIST_](#todo-list)
  - [_Dependence_](#dependence)
    - [MoveIt 1 Source Build: Linux](#moveit-1-source-build-linux)
      - [Prerequisites](#prerequisites)
      - [Create Workspace and Source](#create-workspace-and-source)
      - [Download Source Code](#download-source-code)
      - [Install Moveit](#install-moveit)
    - [Octomap](#octomap)
    - [OpenRave](#openrave)
    - [TOPP-RA](#topp-ra)
      - [Building](#building)
    - [matplotlib-cpp](#matplotlib-cpp)
  - [_Some Reference_](#some-reference)

<!-- /code_chunk_output -->



## _TODO LIST_
- [x] Write brief guidance for this project
- [x] Detailed introduction of `How to build an octomap`.
- [x] Detailed introduction of `How to use JPS Planner`.
- [x] Detailed introduction of `How to build new model and use it`.

## _Dependence_
###  MoveIt 1 Source Build: Linux
You need to install `Moviet` from source. The following build instructions support in particular:
- Ubuntu 20.04 / ROS Noetic
- Ubuntu 18.04 / [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
- Ubuntu 16.04 / ROS Kinetic (no longer officially supported)

#### Prerequisites
- **Install** ROS Melodic
  Follow all the instructions to [install ROS](https://wiki.ros.org/melodic/Installation/Ubuntu). Please make sure you have followed all steps and have the latest versions of packages installed. **ros-melodic-desktop-full** version is recommended.

  Source installation requires [wstool](http://wiki.ros.org/wstool), [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/), and optionally [clang-format](https://clang.llvm.org/docs/ClangFormat.html):
  ```shell
  sudo apt install python-wstool python-catkin-tools clang-format-10 python-rosdep
  ```
#### Create Workspace and Source
  Optionally create a new workspace, you can name it whatever:
  ```shell
  mkdir ~/ws_moveit
  cd ~/ws_moveit
  ```
  Next, source your ROS workspace to load the necessary environment variables, depending on what version of ROS you installed.
  ```shell
  source /opt/ros/melodic/setup.bash
  ```
#### Download Source Code
  Use the following command to build older releases from source:
  ```shell
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall
  wstool update -t src
  rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin config --blacklist \
    moveit_chomp_optimizer_adapter \
    moveit_planners_chomp \
    chomp_motion_planner # Disable CHOMP Motion Planner
  sudo catkin build # sudo will resolve many permission problems
  ```
  you can add this to your `.bashrc` (recommended):
  ```shell
  source ~/ws_moveit/devel/setup.bash
  ```
#### Install Moveit
  After you successfully build the `Moveit` package, you need to install `Moveit`. But `devel` and `install` share the same namespace. If you just config the install option and build, you will find something error if you want to use `Moveit`. Following steps will solve this contradiction.
  ```shell
  sudo catkin clean -b # clean the `build` file
  catkin config --install # if you want to cancel the installation, just replace --install with --no-install
  sudo catkin build
  echo $ROS_PACKAGE_PATH # result: /path/to/mypkg:/opt/ros/melodic/share GOOD!
  echo $ROS_PACKAGE_PATH #result: /opt/ros/melodic/share BAD!
  ```

  I can describe the problem a little more precisely: 
  - when the workspace is configured for `install`, and the `devel` space is rebuilt, then `source devel/setup.bash` gives an incorrect `$ROS_PACKAGE_PATH`. If the workspace is configured for install after the `devel` space is already built, then `source devel/setup.bash` will give the correct `$ROS_PACKAGE_PATH` until the devel space is rebuilt.

### Octomap
Install all package of octomap will help you avoid tons of trouble.
```shell
sudo apt-get install ros-melodic-octomap*
```

### OpenRave
This [script](https://github.com/crigroup/openrave-installation) is an easy way to install the most suitable `OpenRave`, what you need to do is to clone this `git` file and run the following code in that file with terminal.
```shell
git clone https://github.com/crigroup/openrave-installation.git
cd openrave-installation
```
Run the scripts in the following order:
```shell
./install-dependencies.sh
./install-osg.sh
./install-fcl.sh
./install-openrave.sh
```

### TOPP-RA
[huangpham](https://github.com/hungpham2511/toppra) put forward this trajectory planning algorithm. `toppra` is a library for computing the time-optimal path parametrization for robots subject to kinematic and dynamic constraints. In general, given the inputs:

  1. a geometric path `p(s)`, `s` in `[0, s_end]`;
  2. a list of constraints on joint velocity, joint accelerations, tool Cartesian velocity, et cetera.

`toppra` returns the time-optimal path parameterization: s_dot (s), from which the fastest trajectory q(t) that satisfies the given constraints can be found.

In this project we use the `c++` implementation version of TOPP-RA. So you need to check the `./cpp` file and read the `README.md` to install the elemental packages.
#### Building
```shell
# clone
git clone -b develop https://github.com/hungpham2511/toppra

# build
mkdir -p cpp/build && cd cpp/build
cmake ..
make -j4

# run test
./tests/all_tests
```
**[NOTE]: INSTALL ALL OTHER PACKAGES FOLLOWING THE GUIDANCE OF TOPP-RA `./cpp` folder.**

### matplotlib-cpp
[matplotlib-cpp](https://github.com/lava/matplotlib-cpp) is built to resemble the plotting API used by Matlab and matplotlib.
```shell
sudo pip3 install matplotlib
git clone https://github.com/lava/matplotlib-cpp.git
```
Before you use your matplotlib, please write the header `#include "matplotlibcpp.h"`.

## _Some Reference_
> :link: [Realsense2_Description](https://github.com/issaiass/realsense2_description) \
> :link: [realsense_gazebo_description](https://github.com/m-tartari/realsense_gazebo_description) \
> :link: [Intel RealSense Gazebo ROS plugin](https://github.com/m-tartari/realsense_gazebo_plugin) \
> :link: [gazebo2rviz](https://github.com/andreasBihlmaier/gazebo2rviz) \
> :link: [ikfastpy](https://github.com/andyzeng/ikfastpy) \
> :link: [MRSL Jump Point Search Planning Library v1.1](https://github.com/KumarRobotics/jps3d)
