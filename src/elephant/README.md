# Module Manual
This manual will introduce how to use the JPS planning module if you want to change the robot that the original project default model. You need to check the robot type to ensure satisfying `6-DOF` requirement.

---
## 1. robot description
You need to create your own robot description package to describe the robot that you want to use in `URDF` file. `xacro` is a macro assisting to describe the robot more efficiently.

You need to use `Solidworks` to build the 3D model of the robot, and create the `DH` model. `Solidworks` will assist to getting the robot links'interia and generate `STL` type files included in `mesh` folder. 

**[Note]:** You need to take attention that the `unit` in `SolidWorks` often to be `mm`, but in `rviz` and `gazebo` often `m`. Generally we need to scale the `STL` file `0.001` times.

##### USAGE:
I have create a ros package named `elerobot_display`. You can change the codes in the launch file corresponding to what you modified in other folders. Generally you can use `elerobot_display` to debug you robot model, building `TF` relation, improving model, adding new modules etc.
```shell
  roslaunch elerobot_display display_xacro
```

## 2. gazebo world
gazebo world is a simulated physical world. I have create the `elerobot_gazebo_world` package to help you display the gazebo world. By default, the world is empty. You can load the world you have created in `world` folder. Besides, you can display the robot you have created in `elerobot_description`. 

Bear in mind if you want to use the world out of debugging conditions, the world need to be clean without any robot you want to control.

##### USAGE:
```shell
  roslaunch elerobot_gazebo_world elerobot_world.launch 
```
**[Note]:** Details are shown in the files located at `elerobot_gazebo_world` package.

## 3. IKFast

The tutorial [IKFast Kinematics Solver](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html) written for ROS Melodic Moveit1 will help you understand the function of IKFast and know how to install it. But you have to install the necessary modules introduced in [Master-sx](https://github.com/Master-sx)/[ROKAE-deficient-dof-manipulator](https://github.com/Master-sx/ROKAE-deficient-dof-manipulator). I just write how I create a IKFast solver for `Elephant Robot Panda P3`. 

**IKFast** will create `IKFast Solver` according to your description in `xacro` file. The initial state, the joints limitations and other elements will all be taken into consideration. Pleas check your `xacro` file again to guarantee all those parameters correct.

##### Generate .dae file
```shell
export MYROBOT_NAME="elerobot" # the same as the elerobot.urdf.xacro prefix
roscd elerobot_description && cd xacro
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro # convert `xacro` to `urdf`
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae # sudo apt-get install ros-melodic-collada-urdf if "[rospack] Error: package ‘collada_urdf’ not found " occurs
openrave-robot.py "$MYROBOT_NAME".dae --info links # display the link info
```
##### Generate IK Solver
```shell
export PLANNING_GROUP="manipulator" # don't change it !!!
export BASE_LINK="0"
export EEF_LINK="8"
export BASE_LINK_NAME="world"
export EEF_LINK_NAME="elerobot_link7"
export IKFAST_OUTPUT_PATH=$HOME/ws_catkin_elephant/src/elephant/elerobot_ikfast/ikfast61_"$PLANNING_GROUP".cpp
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"
```
If the terminal output end with `store solution`, the solver is generated successfully. Otherwise, you need to check `DH model` first and then the adjust the parameters in `elerobot_description` package.

##### Create Plugin
```shell
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
cd ~/ws_catkin_elephant/src
catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"
cd ..
catkin_make
# After you create the Moveit config package, you can run this command to update yaml file and use IKFast solver in Moveit.
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$BASE_LINK_NAME" "$EEF_LINK_NAME" "$IKFAST_OUTPUT_PATH"
```
## 4.Moveit
This [tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) will help you create the moveit config file. Don't forget to name the config file `elerobot_moveit_config`. `elerobot` is `MYROBOT_NAME` when you create the ikfast solver.

```shell
roslaunch moveit_setup_assistant setup_assistant.launch
```
### moveiet_setup_assistant
---
##### Self-Collision
Higher density leads to more accurate self-collision checking matrix.

##### Virtual Joints
We have set the base link name as `world`, so we don't need to set the `Virtual Joints` again. The `Moveit` will help us to do this.

##### Planning Group
- We will first add Elephant Panda arm as a planning group
  - Enter **Group Name** as `manipulator`
  - Choose `kdl_kinematics_plugin/KDLKinematicsPlugin` as the kinematics solver. We will change it to `IKFast` later. Or if you find `elerobot_manipulator/IKFastKinematicsPlugin`, you can choose it.
  - Let Kin. `Search Resolution` and `Kin. Search Timeout` stay at their default values.
- Now, click on the Add Joints button. 
  - Add `elerobot_joint` 0 to 5.
  - Click `Save`.
- We will also add a group for the end effector. NOTE that you will do this using a different procedure than adding the arm.
  - Click on the **Add Group** button.
  - Enter **Group Name** as `hand`.
  - Let Kin. `Search Resolution` and `Kin. Search Timeout` stay at their default values.
  - Click on the **Add Links** button.
  - Add `elerobot_link7`.
  - Click `Save`.
- Add the `hand` group as a `manipulator` subgroup.

##### Robot Poses
We can set several poses for debug.

##### End Effector
- Choose `hand` as the **End Effector Name** for the gripper.
- Select `hand` as the **End Effector Group**.
- Select `elerobot_link7` as the `Parent Link` for this end-effector.
- Choose `manipulator` as Parent Group.

##### Passive Joints
blank

##### Controllers
Click auto generate button.

##### Author Information
Suit yourself

#### Update Solver
If everything goes well, please enter the following command to update the solver used in `Moveit`:
```shell
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$BASE_LINK_NAME" "$EEF_LINK_NAME" "$IKFAST_OUTPUT_PATH"
```
Then you can run `roslaunch elerobot_moveit_config demo.launch` to check if the robot be controlled successfully.
- Choose the **Planning Group:** _manipulator_
- Choose the **Start State:** _home_
- Choose the **Goal State:** _forward_ or other poses you set in `moveit_setup_assistant`.
- Click `Plan && Execute` to see whether the manipulator will execute the planning result or not. If it reach the goal successfully, congratulations! Otherwise, you need to check whether your **IK solver** worked well or if you had updated the configurations in `elerobot_moveit_config/config/kinematics.yaml`, it needs to be like this:
  ```yaml
  manipulator:
    kinematics_solver: elerobot_manipulator/IKFastKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
  ```  
![Example](./README_pic/moveit_demo_test_ikfast.png)

### Function test
---
Copy the `rokae_moveit_demo` package from `rokae` project. You can change the package name but remember change the parameters in **CMakeLists.txt** and **package.xml**. 
- delete `rokae_msgs` and change the `rokae_arm_ikfast_manipulator_plugin` to `elerobot_ikfast_manipulator_plugin` in `find_package` and `catkin_package` space in **CMakeLists.txt** and **package.xml**.
- If you change the package name, change the **<name>** in both files mentioned above.

`rosrun <PACKAGE_NAME> <TYPE_NAME>` after you have launch the `demo.launch` in moveit config file.

## 5. Intel RealSense i435
Copy the `realsense_gazebo_description` and `realsense_gazebo_plugin` packages from `rokae` project. Now we need to create our sensor camera for manipulator.

- Please run `catkin_make` first to guarantee the ROS can recognize the `realsense_gazebo_description` and `realsense_gazebo_plugin` packages.
- You need to copy the following code and add them to the `<YOUR_ROBOT_DESCRIPTION>.xacro`(`elerobot.xacro`) marco space. And make sure the `with_camera` is `true` in upper file.
  ```xml
      <!-- di435  frame definition can be found at https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md -->
      <xacro:if value="${with_camera}">
        <xacro:include filename="$(find realsense_gazebo_description)/urdf/_d435i.urdf.xacro"/>
        <xacro:sensor_d435i  parent="${arm_id}_link7" name="D435i_camera" topics_ns="D435i_camera" enable_pointCloud="true" align_depth="false">
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </xacro:sensor_d435i> 
      </xacro:if>
  ```
- Use display package to ensure the camera is adjust to the appropriate position and pose.
<center>
<figure>
  <img src="./README_pic/elerobot_with_camera.png" alt="elerobot_with_camera" width="200" align="center" />
  
  <img src="./README_pic/elerobot_realsense_i435.png" alt="elerobot_with_camera" width="200" align="center" />
</figure>
</center>

- Now we need to make the camera work. 
  - Find `elerobot_moveit_config/config/sensors_3d.yaml` file and you will find nothing. That is appropriate. We didn't configure it when we use `moveit_setup_assistant`.
  - Copy the `rokae_arm_moveit_config/config/sensors_3d_point_cloud.yaml` content to `sensor_3d.yaml` and then we rename the `sensor_3d.yaml` to `sensors_3d_point_cloud.yaml`.
  - Next you need to find the `elerobot_moveit_config/launch/move_group.launch`. If you want to open sensor function, what you need to do is re-annotate the **Sensors Functionality** part. We can find the **Sensors Functionality** use the `sensor_manager.launch.xml` file.
  - Modified the code below **Params for the octomap monitor**:
    ```xml
      <!-- Params for the octomap monitor -->
      <param name="octomap_resolution" type="double" value="0.01" />
      <param name="octomap_frame" type="string" value="world" />
      <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
        <!-- The frame_id of the point cloud has to be the sensor frame -->
        <param name="resolution" type="double" value="0.01"/>
        
        <param name="frame_id" type="string" value="world" />
        <!-- maximum range to integrate (speedup!) -->
        <param name="sensor_model/max_range" value="3.0" />
        <param name="latch" value="false"/>
        <!-- data source to integrate (PointCloud2) -->
        <!-- <remap from="cloud_in" to="/$(arg camera_name)/depth/color/points" /> -->
        <remap from="cloud_in" to="/move_group/filter_cloud" />
        <remap from="octomap_point_cloud_centers" to="/$(arg camera_name)/octomap_point_cloud_centers"/>
      </node>
    ```
    And you need to change the 3D sensor config file name to what you have changed. In `elerobot`, these codes are:
    ```xml
    <rosparam command="load" file="$(find elerobot_moveit_config)/config/sensors_3d_point_cloud.yaml" />
    ```
    And you need to add the following codes at beginning:
    ```xml
    <arg name="camera_name" default="D435i_camera"/>
    ```

## 6.Rviz and Gazebo

In this session we will connect the **Rviz** and **Gazebo**. 
Please see the [README.md](elerobot_moveit_config/README.md) in `elerobot_moveit_config` package first and create the necessary files. But I find the new version `Moveit` modified some files name.

---
#### 1. Trajectory Execution Functionality
In `elerobot_moveit_config/launch/move_group.launch`, the **Trajectory Execution Functionality** write:
```xml
  <!-- Trajectory Execution Functionality -->
  <include ns="move_group" file="$(dirname)/trajectory_execution.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_manage_controllers" value="true" />
    <arg name="moveit_controller_manager" value="$(arg moveit_controller_manager)" />
    <arg name="fake_execution_type" value="$(arg fake_execution_type)" />
  </include>
```
- The `moveit_controller_manager` argument becomes `simple`. 
- Next, I check the `trajectory_execution.launch.xml` file, I find that the original `<ROBOT_NAME>_moveit_controller_manager.launch.xml` file becomes `simple_moveit_controller_manager.launch.xml`.
- When digging out the `simple_moveit_controller_manager.launch.xml` I find:
  ```xml
  <rosparam file="$(find elerobot_moveit_config)/config/simple_moveit_controllers.yaml" />
  <rosparam file="$(find elerobot_moveit_config)/config/ros_controllers.yaml" />
  ```
  becomes 
  ```xml
  <rosparam file="$(find rokae_arm_moveit_config)/config/controllers_gazebo.yaml"/>
  ```
  `controllers_gazebo.yaml` corresponds to `simple_moveit_controllers.yaml`. So we modified `simple_moveit_controllers.yaml` to be the same as the `controllers_gazebo.yaml`.
- The `ros_controllers.yaml` corresponds to the file that will be introduced later.

#### 2. ros_controllers.yaml 

---
Copy the `ros_controllers.yaml` in `rokae` moveit configuration package under `config` folder. And you need to change some parameters in it.I find the **Moveit** updates it as well! So let us to dig out the where the `ros_controllers.yaml` used.

- In current version Moveit, I find the `gazebo.launch` contains it:
  ```xml
  <!-- Load ROS controllers -->
  <include file="$(dirname)/ros_controllers.launch"/>
  ```
  The `ros_controllers.launch` uses the `ros_controllers.yaml`. `gazebo.launch` is used in `demo_gazebo.launch`

#### 3. _rokae_simple_control_ package

---
Copy the `rokae_simple_control` package. 

##### 1.rokae_arm_trajectory_controller.launch
I check the `rokae_arm_trajectory_controller.launch` first.

The following command loads the trajectory controller. 
```xml
<rosparam file="$(find rokae_simple_control)/config/rokae_arm_trajectory_control.yaml" command="load"/>
```

And this configuration file has been created by Moveit in `elerobot_moveit_config/config/simple_moveit_controllers.yaml`. If we do not set the prefix namespace like `rokae_arm` in `rokae_arm_trajectory_control.yaml` before `manipulator_controller`, we can write the `simple_moveit_controller_manager.launch.xml` like this:

```yaml
controller_manager_ns: controller_manager
controller_list:
  - name: manipulator_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: True
    joints:
      - elerobot_joint0
      - elerobot_joint1
      - elerobot_joint2
      - elerobot_joint3
      - elerobot_joint4
      - elerobot_joint5
```

- This file `rokae_arm_trajectory_control.yaml` actually corresponds to `elerobot_moveit_config/config/ros_controllers.yaml`. So change the controller to position controller and then do nothing! So we can adjust the codes above to this:
  ```xml
  <rosparam file="$(find elerobot_moveit_config)/config/ros_controllers.yaml" command="load"/>
  ```
- And we have to delete the `ns` in the following codes:
  ```xml
  <node name="arm_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" 
  output="screen" ns="/rokae_arm" args="manipulator_controller"/>
  ```
- And now we can copy the `rokae_pick_place_with_vacuum_gripper` and don't change anything.

##### 2. rokae_arm_gazebo_states.launch

  ```xml
    <rosparam file="$(find rokae_simple_control)/config/rokae_arm_gazebo_joint_states.yaml" command="load"/>
  ```
- The code above will load the `rokae_arm_gazebo_joint_states.yaml` in `config` folder. But it is identical with the `elerobot_moveit_config/config/gazebo_controllers.yaml`. So we change it like this:

  ```xml
    <rosparam file="$(find elerobot_moveit_config)/config/gazebo_controllers.yaml" command="load"/>
  ```
- Delete the `ns` in:
  ```xml
    <node name="joint_controller_spawner" pkg="controller_manager" type="spawner" respawn="true"
  output="screen" ns="/rokae_arm" args="joint_state_controller" />
  ```
- Delete
  ```xml
  <rosparam param="source_list">[/rokae_arm/joint_states]</rosparam>
  xml

##### 3.rokae_arm_gazebo_world.launch
- Delete `tf2_ros`.
- Modify the necessary parameters.

##### 4.rokae_arm_toplevel_moveit.launch
- Modify the necessary parameters.
- We have not created `octomap`, so annotate it first. If we want to load a local `octomap` annotate it again.
- 
- Copy the `moveit_planning_execution.launch` and paste it in `elerobot_moveit_config/launch` and modify some parameters to fit the name of current robot configuration.