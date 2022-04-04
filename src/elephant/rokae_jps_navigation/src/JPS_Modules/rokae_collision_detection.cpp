#include "JPS_Modules/rokae_collision_detection.hpp"
#include "rokae_ikfast_wrapper.cpp"

// [Important Ref] https://blog.csdn.net/u013745804/article/details/79158111?spm=1001.2101.3001.6650.16&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-16.no_search_link&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-16.no_search_link
/*                 https://answers.ros.org/question/356183/how-to-use-the-planning-scene-monitor-in-c/
利用机器人在参数服务器上的robot_description来定义一个planning_scene，可惜的是，
这个planning_scene并不知道碰撞物体的存在，因为它并没有与MoveGroup通信，所以此时碰撞检测是错的。
这里我们先定义了一个PlanningSceneMonitorPtr (psm)，它实际上就是PlanningSceneMonitor*，
注意，这里我们是用robot_description进行初始化的，所以它知道机器人的存在。然后我们利用requestPlanningSceneState()
来启用get_planning_scene服务（为什么要启用这个服务？我们的 LockedPlanningSceneRW 其实可以等价于 psm->getPlanningScene()）。
接着定义一个LockedPlanningSceneRW，它可以看作是对于MoveGroup中动态场景的一次拍照，
为什么要Locked，因为如果我们的场景随着MoveGroup中的场景变化，那么可能会产生一系列的问题，比如如果我们后面想要创建另一个场景，
那么这里的场景也会变化，所以还是使用Locked比较好，这同时也是为什么我们要decoupleParent()的原因。
ps->diff()就是创建一个副本，然后我们调用decoupleParent()将副本剪下来。
*/

// [planning_scene_monitor::LockedPlanningSceneRW] http://docs.ros.org/en/indigo/api/moveit_ros_planning/html/classplanning__scene__monitor_1_1LockedPlanningSceneRW.html
// [robot_state::RobotState] http://docs.ros.org/en/groovy/api/moveit_core/html/classrobot__state_1_1RobotState.html
// [planning_scene_monitor] http://docs.ros.org/en/jade/api/moveit_ros_planning/html/namespaceplanning__scene__monitor.html
// [planning_scene::PlanningScene] http://docs.ros.org/en/indigo/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
// [moveit::core::RobotState] http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
// [Moveit! Tutorial ] https://ros-planning.github.io/moveit_tutorials/doc/planning_scene_monitor/planning_scene_monitor_tutorial.html

// [something useful] https://answers.ros.org/question/281929/collision-checking-using-moveit/


/* Cartesian space planning */
self_detector::self_detector(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{  
  // determine the planning group for processing
  PLANNING_GROUP = "manipulator";

  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer);
  // psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  psm->startSceneMonitor();
  psm->startStateMonitor();
 
  ros::service::waitForService("/get_planning_scene");
  if(psm->requestPlanningSceneState("/get_planning_scene")) {
    psm->startSceneMonitor("/move_group/monitored_planning_scene");
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();
  }
  else {
    ROS_ERROR_STREAM("Error in setting up the PlanningSceneMonitor.");
  }

  // call startSceneMonitor, startWorldGeometryMonitor and startStateMonitor to fully initialize the planning scene monitor
  // /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly */
  // psm->startSceneMonitor("/move_group/monitored_planning_scene");
  // /* listens to changes of world geometry, collision objects, and (optionally) octomaps */
  // psm->startWorldGeometryMonitor();
  // /* listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly. */
  // psm->startStateMonitor();

  // bool success = psm->requestPlanningSceneState("get_planning_scene");
  // ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
  detector_service = nh_.advertiseService("/rokae_arm/robot_collision_detection_node", &self_detector::m_collision_detection, this);

  ROS_INFO("[rokae_collision_detection]: Service On. Ready to do robot collision detection.");

}

// calculate the Quaternion from Rotation Matrix
Eigen::Quaternionf self_detector::m_rotationMatrix2Quaternionf(Eigen::Matrix3f &R)
{  
  Eigen::Quaternionf q(R);
  q.normalize();

  return q;
} 

// If we calculate the IK, when solution found, IK solver will return a vector who is the infomation of matrix T
Eigen::Matrix3f self_detector::m_getRotationMatrix(const std::vector<float> &eef_pose)
{
  Eigen::Matrix3f matrix;
  int nop = 0;
  for (int row = 0; row < 3; row ++)
  {
    for (int col = 0; col < 3; col++)
    {
      matrix(row, col) = eef_pose.at(row*3+col+nop);
    } 
    nop++; // eef_pose is T = [R,t]
  }
  
  return matrix;
}

std::vector<float> self_detector::getTranslation(const std::vector<float> &eef_pose) {
  std::vector<float> transl;
  for (int i = 0; i < 3; i++) {
    transl.push_back(eef_pose.at(i*4+3));
  }

  return transl;
}

bool self_detector::m_collision_detection(rokae_jps_navigation::CheckCollision::Request &req, rokae_jps_navigation::CheckCollision::Response &res)
{
  // This is a convenience class for obtaining access to an instance of a locked PlanningScene.
  // We need to lock the planning scene to avoid the influence of the change of the planning scene

/* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
   for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
   RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */

  planning_scene_monitor::LockedPlanningSceneRW ps(psm); // usage is similar as 'planning_scene::PlanningScene'

  // getCurrentStateNonConst() return a robot state class
  ps->getCurrentStateNonConst().update(); // Update all transforms.

  scene = ps->diff(); // Return a new child PlanningScene that uses this one as parent.
  
  // Make sure that all the data maintained in this scene is local. 
  // All unmodified data is copied from the parent and the pointer to the parent is discarded.
  scene->decoupleParent();

  

  // Get the kinematic model for which the planning scene is maintained.
  const robot_model::RobotModelConstPtr& model = scene->getRobotModel();
  
  // kinematic_state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(model));

  /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
   group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
  const robot_model::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);

  /******************************** debug code ************************************/
  // std::vector<std::string>            joint_names       = joint_model_group->getVariableNames();
  // std::vector<std::string> joint_names {"rokae_arm_joint0", "rokae_arm_joint1", "rokae_arm_joint2", 
  //                                                                           "rokae_arm_joint3", "rokae_arm_joint4", "rokae_arm_joint5"};
  // printf("joint names: \njoint1: %s\njoint2: %s\njoint3: %s\njoint4: %s\njoint5: %s\njoint6: %s\n", joint_names[0].c_str(), joint_names[1].c_str(), joint_names[2].c_str(), joint_names[3].c_str(), joint_names[4].c_str(), joint_names[5].c_str());
  /********************************************************************************/

  std::vector<float> ee_pose {(float)req.path_pose.position.x, (float)req.path_pose.position.y, (float)req.path_pose.position.z, 
  (float)req.path_pose.orientation.w, (float)req.path_pose.orientation.x, (float)req.path_pose.orientation.y, (float)req.path_pose.orientation.z};

  /******************************** debug code ************************************/
  // std::vector<float> ee_pose {0.554500, 0, 1.053000, 0.0, 0.707107, 0.0, 0.707107};
  // std::vector<float> eef_joint_configs{0, 0, 0, 0, 0, 0};
  /********************************************************************************/

  std::vector<float> prev_joint_configs {(float)req.prev_joints.at(0), (float)req.prev_joints.at(1), (float)req.prev_joints.at(2), 
                                        (float)req.prev_joints.at(3), (float)req.prev_joints.at(4), (float)req.prev_joints.at(5)};

  std::shared_ptr<robots::Kinematics> _IKsolver(new robots::Kinematics());
  std::vector<float>                                                           joint_state_values = _IKsolver->inverse(ee_pose);
  std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> ik_result          = _IKsolver->getClosestIK(joint_state_values, prev_joint_configs);

  // pair the 'divided_joint_configs', 'effective_num' and 'min_index'
  int min_index = ik_result.second.second;
  if (min_index == -1) // min_index, if min_index = -1, which means there's no solution for current pose
  {
    if (req.ifVerbose) {
      printf(ANSI_COLOR_MAGENTA "No solution for target pose." ANSI_COLOR_RESET "\n");
    }
    res.getSolution = false;  // no solution
    res.isCollide   = true;   // also set collide
    res.curr_joints.clear();  // clear the curr_joints for response message

    return true;
  } // if we get at least one effective IK solution for current pose, we continue to process further

  // convert data type
  std::vector<int>                effective_index (ik_result.second.first.begin(), ik_result.second.first.end()); // all effective index
  std::vector<std::vector<float>> divided_joint_configs (ik_result.first.begin(), ik_result.first.end());         // all joint-configs solutions
  std::vector<double>             joint_positions (divided_joint_configs.at(min_index).begin(), divided_joint_configs.at(min_index).end()); 

  /******************************** debug code ************************************/
  // ROS_INFO("CURRENT JOINT CONFIGS: (%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)\n", 
  //                     joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5]);
  /********************************************************************************/

  // We have got IK solution for current pose. We need to check if this state is valid. 
  res.getSolution = true;
  res.curr_joints.clear();

  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult  collision_res;

  /*********************** collision detection parament setting ********************************/
  // calculate collision distance
  // [WARNING]: THIS WILL SLOW THE DETECTION SPEED GRAMMATICALLY 
  // collision_req.distance = true;

  // if you want to print the detection result
  // collision_req.verbose  = true;
  
  // collision_req.contacts = true;

  // choose the collision detection group 
  // !!!!!!!!!!!!!!!!!!! I set this parament and the result seems to be right. !!!!!!!!!!!!!!!!!!!!!!!!!
  collision_req.group_name     = PLANNING_GROUP; 

  /* print out useful message */
  // scene->printKnownObjects(std::cout);
  // kinematic_state->printStatePositions(std::cout);

  /* robot state has been updated, so we get it again to check collision. 
     Set the joints'value the manipulator assumed to be. */
  // for (std::size_t i = 0; i < joint_names.size(); ++i) {
  //   kinematic_state->setJointPositions(joint_names[i].c_str(), &joint_positions[i]);
  // }
  // kinematic_state->setVariablePositions(joint_names, joint_positions);
  kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
  // clear previous collision detection result
  collision_res.clear();
  // check the self collision and Env collision
  scene->checkCollision(collision_req, collision_res, *kinematic_state);
  // scene->checkSelfCollision(collision_req, collision_res);
  // printf(ANSI_COLOR_GREEN "collision distance: %f" ANSI_COLOR_RESET"\n", collision_res.distance);

  // response variable store the result contained in collision_result.collision
  res.isCollide = collision_res.collision;
  if(!res.isCollide) {
    res.curr_joints.assign(joint_positions.begin(), joint_positions.end());
    if (req.ifVerbose) {
      std::cout << ANSI_COLOR_MAGENTA "Current state is " << (res.isCollide ? "in" : "not in") << " collision" ANSI_COLOR_RESET << std::endl; 
    }
    return true;
  } // no collision
  

  // previous condition may change the state of 'res.isCollide', if 'res.isCollide is true', we will do further process.
  // find the non-collision state in remain effective indexes
  for (int i = 0; i < (int)effective_index.size(); i++)
  {
    int index = effective_index.at(i);
    if (index == min_index) {
      continue;
    } // min_index had been checked

    // update the content in 'joint_positions'
    joint_positions.assign(divided_joint_configs.at(index).begin(), divided_joint_configs.at(index).end());
    /******************************** debug code ************************************/
    // std::vector<double> joint_positions = { 0.0, 0.0, -3.0, 0.0, -1.0, 0.0 };
    /********************************************************************************/

    // set joint positions for robot
    // kinematic_state->setVariablePositions(joint_names, joint_positions);

    // robot state has been updated, so we get it again to check collision
    // for (std::size_t i = 0; i < joint_names.size(); ++i) {
    //   kinematic_state->setJointPositions(joint_names[i].c_str(), &joint_positions[i]);
    // }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);

    collision_res.clear();
    scene->checkCollision(collision_req, collision_res, *kinematic_state);
    // scene->checkSelfCollision(collision_req, collision_res);
    // printf(ANSI_COLOR_GREEN "collision distance: %f" ANSI_COLOR_RESET"\n", collision_res.distance);

    res.isCollide = collision_res.collision;
    if(!res.isCollide)
    {
      res.curr_joints.assign(joint_positions.begin(), joint_positions.end());
      break;
    }
  } // check the remaining joint states, until no collision state is found or all states checked.

  if (req.ifVerbose) {
    std::cout << ANSI_COLOR_MAGENTA "Current state is " << (res.isCollide ? "in" : "not in") << " collision" ANSI_COLOR_RESET << std::endl; 
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rokae_collision_detection");
  ros::AsyncSpinner spinner(0); 
  spinner.start();

  ros::NodeHandle nh_srv;
  self_detector coll_det(&nh_srv);

  ros::waitForShutdown();
  return 0;
}



  // robots::Kinematics* IKsolver_test(new robots::Kinematics());
  // std::vector<float> ee_pose_test {0.554500, 0, 1.053000, 0.0, 0.707107, 0.0, 0.707107};
  // std::vector<float> ee_pose_test2 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // // std::vector<float> joint_state_values_test = IKsolver_test->forward(ee_pose_test2);
  // // std::vector<float> joint_state_values_test = IKsolver_test->inverse(ee_pose_test);

  // std::vector<float> ee_pose_pre {0, 1, 0, 0, 0, 0};
  // const std::vector<float> tmp = IKsolver_test->inverse(ee_pose_test);
  // std::vector<float> joint_state_values_test = IKsolver_test->getClosestIK(tmp, ee_pose_pre);

  // // Eigen::Matrix3f eef_rotation = getRotationMatrix(joint_state_values_test);
  // // Eigen::Quaternionf q = rotationMatrix2Quaterniond(eef_rotation);
  // // ROS_INFO_STREAM("Quaterniond:(" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ")");

  // if (joint_state_values_test.size() == 0)
  // {
  //   ROS_ERROR("No solution");
  // }

  // std::string joint_state_string_test; // Joints Configration Output
  // for (const auto & item : joint_state_values_test){
  //   joint_state_string_test += std::to_string(item);
  //   joint_state_string_test += " ";
  // }
  // // ROS_INFO_STREAM("Joints Configration at this point: " << joint_state_string_test);
  // ROS_INFO_STREAM("eef state: " << joint_state_string_test);

  // void self_detector::searchPolicy(const std::vector<float> prev_joint_configs)
// {
//   // Monte Carlo Search
//   robots::Kinematics* _IKsolver(new robots::Kinematics());
//   std::vector<float> pose {0.554500, 0, 1.053000};
//   std::vector<float> eef_joint_configs {0, 0, 0, 0, 0, 0};
//   std::vector<float> search_joint_configs(6);
//   float tolerance = 0.02;
//   for (int n = 5000; n > 0; n--)
//   {
//     search_joint_configs.clear();
//     for (int i = 0; i < 6; i++)
//     {
//       search_joint_configs.push_back(prev_joint_configs.at(i) + (rand()% 100 - 50 )*0.01);
//     }
//     bool success = _IKsolver->CheckBound(search_joint_configs);

//     if (success)
//     {
//         std::vector<float> test_pose = _IKsolver->forward(search_joint_configs);
//         std::vector<float> transl = getTranslation(test_pose);
//         double x_err = abs(transl[0] - pose[0]);
//         double y_err = abs(transl[1] - pose[1]);
//         double z_err = abs(transl[2] - pose[2]);
//         double cost = pow()
//         if ( x_err <= tolerance &&  y_err <= tolerance && z_err <= tolerance)
//         {
//           ROS_INFO_STREAM("Get joint configs for target point.");
//           ROS_INFO_STREAM("Joint Configs: (" << search_joint_configs[0] << "," << search_joint_configs[1] << "," << search_joint_configs[2] << ","
//           << search_joint_configs[3] << "," << search_joint_configs[4] << "," << search_joint_configs[5] << ")");
//           break;
//         }
//     }
//     ROS_INFO_STREAM(n);
//   }
  
// }