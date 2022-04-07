# Changelog

## Branch-Name(main)
- **Branch summary:** Elephant Panda3 `6-DOF` manipulator projects
- **Maintained by:** m-contour <m-contour@qq.com>

### 2022-04-07 
#### Feature
- **Fixed:** (`rokae_octomap_unit/src/rokae_static_octomap_publish_node.cpp`) Change octomap subscriber topic name from `octomap_binary` to `/move_group/octomap_binary`, which solves the problem that the planning scene sometimes cannot display the geometry published by octomap server. **[#841aaab](https://github.com/Master-sx/eleRobotJPS/commit/841aaabc6ad1139cd57a78d33c748139be285d88)**
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