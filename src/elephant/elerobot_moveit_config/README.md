# Communication between Moveit! and Gazebo

There exist some defects in current configuration files generated through **Moveit SetupAssistant 2.0**.

### 1. IKFast Solver
If you want to automatically update the `Solver` in `PACKAGE_NAME_moveit_config`, you need to initially choose a solver. Otherwise, the `config/kinematics.yaml` will be `{}` and the updating process will not be implemented.

### 2. config/controllers_gazebo.yaml
If you want to communicate Gazebo, you need to create a `.yaml` file based on `ros_controllers.yaml`. You need to add the ROBOT_NAME namespace in the `name` property.

What is more, `controller_manager_ns: controller_manager` is needed on account of the requirement of the searching `namespace`.

### 3. launch/moveit_planning_execution.launch
You may copy the format from the website for this file, but the latest version in `PACKAGE_NAME_moveit_config` convert the `config` argument in `moveit_rviz.lanch` to `rviz_config`.

### 4. launch/trajectory_execution.launch.xml
The crucial problem happened in `trajectory_execution.launch.xml` file. 

```xml
<launch>

  <!-- This file makes it easy to include the settings for trajectory execution  -->
  <arg name="fake_execution" default="false"/>
  <arg name="execution_type" default="interpolate" />

  <!-- Flag indicating whether MoveIt! is allowed to load/unload  or switch controllers -->
  <arg name="moveit_manage_controllers" default="true"/>
  <param name="moveit_manage_controllers" value="$(arg moveit_manage_controllers)"/>

  <!-- When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution -->
  <param name="trajectory_execution/allowed_execution_duration_scaling" value="1.2"/> <!-- default 1.2 -->
  <!-- Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling) -->
  <param name="trajectory_execution/allowed_goal_duration_margin" value="0.5"/> <!-- default 0.5 -->
  <!-- Allowed joint-value tolerance for validation that trajectory's first point matches current robot state -->
  <param name="trajectory_execution/allowed_start_tolerance" value="0.01"/> <!-- default 0.01 -->

  <!-- Load the robot specific controller manager; this sets the moveit_controller_manager ROS parameter -->
  <arg name="moveit_controller_manager" default="rokae_arm" />
  <include file="$(find rokae_arm_moveit_config)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml">
    <arg name="execution_type" value="$(arg execution_type)" if="$(arg fake_execution)"/>
  </include>

</launch>
```

This file declare an argument `execution_type`. However, the upper level file which includes this file provokes two choices, `fake` and `ROBOT_NAME`. `execution_type` is only used in `fake`. This file ignore the information propagation flow that `fake_execution` argument value is not transferred to this file. Therefore, `<arg name="fake_execution" default="false"/> ` needs to be added to this file and convert `<arg name="execution_type" value="$(arg execution_type)"/>` to `<arg name="execution_type" value="$(arg execution_type)" if="$(arg fake_execution)"/>`. 

In the upper level file `move_group.launch`, `<arg name="fake_execution" value="$(arg fake_execution)"/>` needs to be added to `<!-- Trajectory Execution Functionality -->` under  `include` definition domain.

### 5. launch/planning_context.launch 

```shell
[ERROR] [1635217620.151383632, 721.685000000]: Transform error: Lookup would require extrapolation into the past.  Requested time 696.311000000 but the earliest data is at time 711.632000000, when looking up transform from frame [rokae_arm_link1] to frame [D435_camera_color_optical_frame]
[ERROR] [1635217620.152151720, 721.685000000]: Transform cache was not updated. Self-filtering may fail. If transforms were not available yet, consider setting robot_description_planning/shape_transform_cache_lookup_wait_time to wait longer for transforms
```

You can use `rosrun tf view_frames` to get the information of the tf process time. The errors raised above is on account of the insufficient `shape_transform_cache_lookup_wait_time`.

 TF keeps transformation data for 10 seconds before discarding it, if the transformation is getting published irregularly it might not be in the TF tree anymore. If the transformation is getting published and received by two different machines make sure their clocks are in sync. 

 ### 6.subgroup
If you follow the moveit official tutorial to setup your robot, you will set the `end_effector` and the `manipulator` in separate group. However, when using `move_group` node to control the robot, I found that the `end_effector` info is empty. One `Github Issues` reflected a similar situation [[REF](https://github.com/ros-planning/moveit/issues/1694)]. I found a more suitable way to add the `end_effector` to the planning group. 

- add the `end_effector` to `subgroup` of your planning group. 
**TIPS:** You need to create the `end_effector` group first, such as `hand`.