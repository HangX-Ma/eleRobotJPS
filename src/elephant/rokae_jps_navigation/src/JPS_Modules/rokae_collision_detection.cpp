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

  // Load the planning scene monitor
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh_);
  psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer);

  if (!psm->getPlanningScene()) {
    ROS_ERROR_STREAM("Error in setting up the PlanningSceneMonitor");
    exit(-1);
  }

  psm->startSceneMonitor();
  psm->startStateMonitor();
  psm->startWorldGeometryMonitor();
  psm->requestPlanningSceneState();

  ros::Duration(0.5).sleep();

  std::vector<std::string> topics;
  psm->getMonitoredTopics(topics);
  ROS_INFO_STREAM(ANSI_COLOR_CYAN "Listening for scene updates on topics " ANSI_COLOR_RESET << boost::algorithm::join(topics, ", "));
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
  pub_aabb         = nh_.advertise<visualization_msgs::Marker>("visualization_aabb", 10);
  pub_obb          = nh_.advertise<visualization_msgs::Marker>("/visualization_obb", 10);


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
  // scene = planning_scene::PlanningScene::clone(ps);

  // construct a planning scene that is just a diff on top of our actual planning scene
  // assume the current state of the diff world is the one we plan to reach

  scene = ps->diff(); // Return a new child PlanningScene that uses this one as parent.
  scene->decoupleParent();


  // Make sure that all the data maintained in this scene is local. 
  // All unmodified data is copied from the parent and the pointer to the parent is discarded.
  // scene->decoupleParent();
  scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  // scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  double radius = 0.02;
  double lifetime = 600.0;
  unsigned int trials = 2; // The number of repeated collision checks for each state

  // Get the kinematic model for which the planning scene is maintained.
  const robot_model::RobotModelConstPtr& model = scene->getRobotModel();
  collision_detection::AllowedCollisionMatrix acm{collision_detection::AllowedCollisionMatrix(model->getLinkModelNames(), true) };
  // kinematic_state
  robot_state::RobotState& kinematic_state = scene->getCurrentStateNonConst();



  /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
   group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
  const robot_model::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(PLANNING_GROUP);
  kinematic_state.setToDefaultValues(joint_model_group, "home");
  kinematic_state.update();
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
  
  collision_req.contacts = true;
  collision_req.max_contacts = 99;
  collision_req.max_contacts_per_pair = 10;
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
  // kinematic_state.setJointGroupPositions(joint_model_group, joint_positions);
  kinematic_state.setJointPositions("elerobot_joint0", &joint_positions[0]);
  kinematic_state.setJointPositions("elerobot_joint1", &joint_positions[1]);
  kinematic_state.setJointPositions("elerobot_joint2", &joint_positions[2]);
  kinematic_state.setJointPositions("elerobot_joint3", &joint_positions[3]);
  kinematic_state.setJointPositions("elerobot_joint4", &joint_positions[4]);
  kinematic_state.setJointPositions("elerobot_joint5", &joint_positions[5]);
  kinematic_state.update();

  scene->setCurrentState(kinematic_state);

  std::vector<double> gstate;
  // scene->getCurrentState().copyJointGroupPositions(PLANNING_GROUP,gstate);
  // ROS_INFO("gstate size: %d\njoint config: %f,%f,%f,%f,%f,%f\n",gstate.size(), gstate[0], gstate[1], gstate[2], gstate[3], gstate[4], gstate[5]);
  std::vector<double> aabb;
  scene->getCurrentState().computeAABB(aabb);

  // Prepare the ROS message we will reuse throughout the rest of the function.
  visualization_msgs::Marker msg;
  msg.header.frame_id = model->getRootLinkName();
  msg.type         = visualization_msgs::Marker::CUBE;
  msg.color.a      = 0.5;
  msg.lifetime.sec = 3000;

  // Publish the AABB of the whole model
  msg.ns = "elerobot";
  msg.pose.position.x = (aabb[0] + aabb[1]) / 2;
  msg.pose.position.y = (aabb[2] + aabb[3]) / 2;
  msg.pose.position.z = (aabb[4] + aabb[5]) / 2;
  msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 1;
  msg.scale.x = aabb[1] - aabb[0];
  msg.scale.y = aabb[3] - aabb[2];
  msg.scale.z = aabb[5] - aabb[4];
  pub_aabb.publish(msg);

  // Publish BBs for all links
  std::vector<const moveit::core::LinkModel*> links = model->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    Eigen::Isometry3d      transform = kinematic_state.getGlobalLinkTransform(links[i]);  // intentional copy, we will translate
    const Eigen::Vector3d& extents   = links[i]->getShapeExtentsAtOrigin();
    transform.translate(links[i]->getCenteredBoundingBoxOffset());
    moveit::core::AABB aabb_all;
    aabb_all.extendWithTransformedBox(transform, extents);

    // Publish AABB
    msg.ns              = links[i]->getName();
    msg.pose.position.x = transform.translation()[0];
    msg.pose.position.y = transform.translation()[1];
    msg.pose.position.z = transform.translation()[2];
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.color.r = 1;
    msg.color.b = 0;
    msg.scale.x = aabb_all.sizes()[0];
    msg.scale.y = aabb_all.sizes()[1];
    msg.scale.z = aabb_all.sizes()[2];
    pub_aabb.publish(msg);

    // Publish OBB (oriented BB)
    msg.ns += "-obb";
    msg.pose.position.x = transform.translation()[0];
    msg.pose.position.y = transform.translation()[1];
    msg.pose.position.z = transform.translation()[2];
    msg.scale.x = extents[0];
    msg.scale.y = extents[1];
    msg.scale.z = extents[2];
    msg.color.r = 0;
    msg.color.b = 1;
    Eigen::Quaterniond q(transform.linear());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pub_obb.publish(msg);
  }

  for (unsigned int i = 0; i < trials; ++i) {
    // clear previous collision detection result
    collision_res.clear();
    // check the self collision and Env collision
    scene->checkCollision(collision_req, collision_res, kinematic_state, acm);
  }

  // scene->checkSelfCollision(collision_req, collision_res);
  // printf(ANSI_COLOR_GREEN "collision distance: %f" ANSI_COLOR_RESET"\n", collision_res.distance);
  // ROS_INFO(ANSI_COLOR_CYAN "obs dist: %f" ANSI_COLOR_RESET, collision_res.distance);

  // color collided objects red
  for (auto& contact : collision_res.contacts)
  {
    // ROS_INFO_STREAM(ANSI_COLOR_CYAN "Between: " << contact.first.first << " and " << contact.first.second << ANSI_COLOR_RESET);
    std_msgs::ColorRGBA red;
    red.a = 0.8;
    red.r = 1;
    red.g = 0;
    red.b = 0;
    scene->setObjectColor(contact.first.first, red);
    scene->setObjectColor(contact.first.second, red);
  }
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
    // kinematic_state.setJointGroupPositions(joint_model_group, joint_positions);
    kinematic_state.setJointPositions("elerobot_joint0", &joint_positions[0]);
    kinematic_state.setJointPositions("elerobot_joint1", &joint_positions[1]);
    kinematic_state.setJointPositions("elerobot_joint2", &joint_positions[2]);
    kinematic_state.setJointPositions("elerobot_joint3", &joint_positions[3]);
    kinematic_state.setJointPositions("elerobot_joint4", &joint_positions[4]);
    kinematic_state.setJointPositions("elerobot_joint5", &joint_positions[5]);
    kinematic_state.update();

    scene->setCurrentState(kinematic_state);
    psm->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE);

    std::vector<double> gstate;
    scene->getCurrentState().copyJointGroupPositions(PLANNING_GROUP,gstate);
    // ROS_INFO("gstate size: %d\njoint config: %f,%f,%f,%f,%f,%f\n",gstate.size(), gstate[0], gstate[1], gstate[2], gstate[3], gstate[4], gstate[5]);

    std::vector<double> aabb;
    scene->getCurrentState().computeAABB(aabb);
    // Prepare the ROS message we will reuse throughout the rest of the function.
    visualization_msgs::Marker msg;
    msg.header.frame_id = model->getRootLinkName();
    msg.type         = visualization_msgs::Marker::CUBE;
    msg.color.a      = 0.5;
    msg.lifetime.sec = 3000;

    // Publish the AABB of the whole model
    msg.ns = "elerobot";
    msg.pose.position.x = (aabb[0] + aabb[1]) / 2;
    msg.pose.position.y = (aabb[2] + aabb[3]) / 2;
    msg.pose.position.z = (aabb[4] + aabb[5]) / 2;
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.scale.x = aabb[1] - aabb[0];
    msg.scale.y = aabb[3] - aabb[2];
    msg.scale.z = aabb[5] - aabb[4];
    pub_aabb.publish(msg);

    // Publish BBs for all links
    std::vector<const moveit::core::LinkModel*> links = model->getLinkModelsWithCollisionGeometry();
    for (std::size_t i = 0; i < links.size(); ++i)
    {
      Eigen::Isometry3d      transform = kinematic_state.getGlobalLinkTransform(links[i]);  // intentional copy, we will translate
      const Eigen::Vector3d& extents   = links[i]->getShapeExtentsAtOrigin();
      transform.translate(links[i]->getCenteredBoundingBoxOffset());
      moveit::core::AABB bounding_box;
      bounding_box.extendWithTransformedBox(transform, extents);

      // Publish AABB
      msg.ns              = links[i]->getName();
      msg.pose.position.x = transform.translation()[0];
      msg.pose.position.y = transform.translation()[1];
      msg.pose.position.z = transform.translation()[2];
      msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
      msg.pose.orientation.w = 1;
      msg.color.r = 1;
      msg.color.b = 0;
      msg.scale.x = bounding_box.sizes()[0];
      msg.scale.y = bounding_box.sizes()[1];
      msg.scale.z = bounding_box.sizes()[2];
      pub_aabb.publish(msg);

      // Publish OBB (oriented BB)
      msg.ns += "-obb";
      msg.pose.position.x = transform.translation()[0];
      msg.pose.position.y = transform.translation()[1];
      msg.pose.position.z = transform.translation()[2];
      msg.scale.x = extents[0];
      msg.scale.y = extents[1];
      msg.scale.z = extents[2];
      msg.color.r = 0;
      msg.color.b = 1;
      Eigen::Quaterniond q(transform.linear());
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      pub_obb.publish(msg);
    }

    for (unsigned int i = 0; i < trials; ++i) {
      // clear previous collision detection result
      collision_res.clear();
      // check the self collision and Env collision
      scene->checkCollision(collision_req, collision_res, kinematic_state, acm);
    }

    // scene->checkSelfCollision(collision_req, collision_res);
    // printf(ANSI_COLOR_GREEN "collision distance: %f" ANSI_COLOR_RESET"\n", collision_res.distance);
    // ROS_INFO(ANSI_COLOR_CYAN "obs dist: %f" ANSI_COLOR_RESET, collision_res.distance );

    // color collided objects red
    for (auto& contact : collision_res.contacts)
    {
      // ROS_INFO_STREAM(ANSI_COLOR_CYAN "Between: " << contact.first.first << " and " << contact.first.second << ANSI_COLOR_RESET);
      std_msgs::ColorRGBA red;
      red.a = 0.8;
      red.r = 1;
      red.g = 0;
      red.b = 0;
      scene->setObjectColor(contact.first.first, red);
      scene->setObjectColor(contact.first.second, red);
    }
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