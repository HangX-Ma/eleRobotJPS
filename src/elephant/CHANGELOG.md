# Changelog

## Branch-Name(main)
- **Branch summary:** Elephant Panda3 `6-DOF` manipulator projects
- **Maintained by:** m-contour <m-contour@qq.com>

### 2022-04-14 
#### Feature
- **Add:** (`elerobot_socket`) Finished socket commands and updated TCP socket communication. **[#271592d](https://github.com/Master-sx/eleRobotJPS/commit/271592de40cdd5578bee93523d1f615f6fb69a1e)**
- **Bug:** `Intel RealSense D435i` Because my computer has no USB 3.0 port, though using Type-C instead, errors still occur. I guess this may be related to the camera itself. Further survey needs to be done. **[#271592d](https://github.com/Master-sx/eleRobotJPS/commit/271592de40cdd5578bee93523d1f615f6fb69a1e)**


### 2022-04-11 
#### Feature
- **Add:** (`elerobot_socket`) Add socket commands. **[#dab54b6](https://github.com/Master-sx/eleRobotJPS/commit/dab54b6501a6a05910517343d2ae22ba320fbc00)**
 
### 2022-04-11 
#### Feature
- **Add:** (`elerobot_simple_control/launch/elerobot_toplevel_moveit.launch`) (`elerobot_moveit_config/config/sensors_3d_point_cloud.yaml`) Local Intel Realsense D435i setting. Update configuration steps in `README.md`. **(not test)** **[#5fea039](https://github.com/Master-sx/eleRobotJPS/commit/5fea03935fd0ed8de0ddb22ea4f48c32a1eb1c32)**
- **Add:** (`elerobot_map_convertor`) ROS package for converting local `.pcd` file to octomap `.bt` file. **(not test)** **[#5fea039](https://github.com/Master-sx/eleRobotJPS/commit/5fea03935fd0ed8de0ddb22ea4f48c32a1eb1c32)**
- **Add:** (`elerobot_socket`) ROS package for elephant Panda3 robot socket control. **(not complete)** **[#5fea039](https://github.com/Master-sx/eleRobotJPS/commit/5fea03935fd0ed8de0ddb22ea4f48c32a1eb1c32)**

### 2022-04-10 
#### Feature
- **Fixed:** (`rokae_jps_navigation`) Fixed the collision detection problem. The planning scene topic is `/planning_scene`. User needs to guarantee the `Rviz`, `elerobot_description`, `rokae_collision_detection.cpp` all use the same **topic**.

### 2022-04-07 
#### Feature
- ~~**Fixed:** (`rokae_octomap_unit/src/rokae_static_octomap_publish_node.cpp`) Change octomap subscriber topic name from `octomap_binary` to `/move_group/octomap_binary`, which solves the problem that the planning scene sometimes cannot display the geometry published by octomap server. **[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**~~
- **Fixed:** (`rokae_jps_navigation`) 
  - In `JPS_Planner/rokae_jps_planner`, add `dynamicEDT3D` octomap distance map package. This pretreatment helps to generate effective octomap and the user can successfully change the node occupancy values. **[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
  - In `JPS_Planner/rokae_jps_planner`, add `prev_joint` container to maintain the previous joint values. Otherwise, toppra will receive previous joints remained by last planning. This will cause error. **[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
  - In `JPS_Planner/rokae_jps_planner` and `JPS_Planner/rokae_graph_search`, delete `keytocoord_modified`, because the `point3d` in octomap is at the center of voxel. This helps to correctly using the octomap.**[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
- **Fixed:** (`rokae_arm_toppra/src/rokae_toppra_server.cpp`) Add `plt::backend("agg");` to solve a raised problem. Change the `length2` to constant value. Otherwise the `toppra` may raise error. **[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
  ```c++
  253 const int length2 = 20;
  ```
- **Add:** (`JPS_Planner/rokae_jps_planner`) Add `suit_coordinate()` function, used to preprocess the start and goals.**[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
- **Add:** (`JPS_Planner/rokae_jps_planner`) Add `toppra` trajectory joint configurations checker. Change the `rokae_joint2pose` service `srv` message.**[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
- **Bug:** (`JPS_Planner/rokae_collision_detection`) Contact between environment and robot without examination.
#### Debug Log

[moveit/moveit_ros/planning/planning_components_tools/src/evaluate_collision_checking_speed.cpp](https://github.com/ros-planning/moveit/blob/cce0ffe58c3f472fc5bf76b1ec364d29d2fa7252/moveit_ros/planning/planning_components_tools/src/evaluate_collision_checking_speed.cpp)
[moveit/moveit_ros/planning/planning_components_tools/src/visualize_robot_collision_volume.cpp](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/planning/planning_components_tools/src/visualize_robot_collision_volume.cpp)
[moveit/moveit_ros/planning/planning_components_tools/src/compare_collision_speed_checking_fcl_bullet.cpp](https://github.com/ros-planning/moveit/blob/779b7c8b019f70898d4de3189f9261c9697d9b9f/moveit_ros/planning/planning_components_tools/src/compare_collision_speed_checking_fcl_bullet.cpp#L92)
[moveit/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp](https://github.com/ros-planning/moveit/blob/cce0ffe58c3f472fc5bf76b1ec364d29d2fa7252/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp)
[moveit/moveit_core/robot_state/test/test_aabb.cpp](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_core/robot_state/test/test_aabb.cpp)
[Collision checking only includes some links from a EEF group with multiple fingertips #2701](https://github.com/ros-planning/moveit/issues/2701)