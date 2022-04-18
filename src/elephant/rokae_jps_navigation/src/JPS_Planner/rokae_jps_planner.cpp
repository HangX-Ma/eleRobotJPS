#include "JPS_Planner/rokae_jps_planner.hpp"

JPSPlanner::JPSPlanner(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
  printf("\n" ANSI_COLOR_BLUE "[rokae_JPS_planner]: Initializing..." ANSI_COLOR_RESET "\n");
  printf(ANSI_COLOR_BLUE "-------------Loading paraments---------------" ANSI_COLOR_RESET "\n");
  bool loaded_successfully = true;

  // planning
  loaded_successfully &= parse_param("/rokae_arm/planning/planning_tree_resolution", planning_tree_resolution_);
  loaded_successfully &= parse_param("/rokae_arm/planning/planning_timeout", planning_timeout_);
  loaded_successfully &= parse_param("/rokae_arm/planning/JPS_timeout", JPS_timeout_);
  loaded_successfully &= parse_param("/rokae_arm/planning/planner_verbose", planner_verbose_);
  loaded_successfully &= parse_param("/rokae_arm/planning/debug_verbose", debug_verbose_);
  loaded_successfully &= parse_param("/rokae_arm/planning/xDim", xDim_);
  loaded_successfully &= parse_param("/rokae_arm/planning/yDim", yDim_);
  loaded_successfully &= parse_param("/rokae_arm/planning/zDim", zDim_);
  loaded_successfully &= parse_param("/rokae_arm/planning/eps", eps_);
  loaded_successfully &= parse_param("/rokae_arm/planning/max_iteration", max_iteration_);
  loaded_successfully &= parse_param("/rokae_arm/planning/JPS_max_iteration", JPS_max_iteration_);
  loaded_successfully &= parse_param("/rokae_arm/planning/search_radius", search_radius_);
  loaded_successfully &= parse_param("/rokae_arm/planning/potential_radius", potential_radius_);
  loaded_successfully &= parse_param("/rokae_arm/planning/potential_weight", potential_weight_);
  loaded_successfully &= parse_param("/rokae_arm/planning/heu_type", heu_type_);
  loaded_successfully &= parse_param("/rokae_arm/planning/euclidean_distance_cutoff", euclidean_distance_cutoff_);
  loaded_successfully &= parse_param("/rokae_arm/default_object_padding", default_object_padding_);

  // visualization 
  loaded_successfully &= parse_param("/rokae_arm/visualization/show_unoccupied", show_unoccupied_);
  loaded_successfully &= parse_param("/rokae_arm/visualization/tree_points_scale", tree_points_scale_);
  loaded_successfully &= parse_param("/rokae_arm/visualization/expansions_points_scale", expansions_points_scale_);
  loaded_successfully &= parse_param("/rokae_arm/visualization/path_points_scale", path_points_scale_);
  loaded_successfully &= parse_param("/rokae_arm/visualization/point_scale", point_scale_);
  loaded_successfully &= parse_param("/rokae_arm/visualization/voxel_scale", voxel_scale_);

  if (!loaded_successfully) 
  {
    printf(ANSI_COLOR_RED "Could not load all non-optional parameters. Shutting down." ANSI_COLOR_RESET "\n");
    ros::shutdown();

    return;
  }

  // publisher
  voxel_publisher_       = nh_.advertise<visualization_msgs::Marker>("/rokae_arm/voxel_marker_out", 1);
  point_publisher_       = nh_.advertise<visualization_msgs::Marker>("/rokae_arm/point_marker_out", 1);
  binary_tree_publisher_ = nh_.advertise<visualization_msgs::Marker>("/rokae_arm/binary_tree_markers_out", 1);
  expansion_publisher_   = nh_.advertise<visualization_msgs::Marker>("/rokae_arm/expansion_markers_out", 1);
  path_publisher_        = nh_.advertise<visualization_msgs::Marker>("/rokae_arm/path_markers_out", 1);
  // subscriber
  octomap_subscriber_    = nh_.subscribe("/octomap_binary", 1, &JPSPlanner::octomapCallback, this);
  // service 
  goto_service_          = nh_.advertiseService("/rokae_arm/goto_trigger", &JPSPlanner::gotoCallback, this);

  currGoal_id_ = 0;
  is_initialized_        = true;
  printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Initialized" ANSI_COLOR_RESET "\n");

}


bool JPSPlanner::setInitialJoints(float j1, float j2, float j3, float j4, float j5, float j6)
{
  if (!check_status_) {
    prev_joint_configs_.clear();
    prev_joint_configs_.push_back(j1);
    prev_joint_configs_.push_back(j2);
    prev_joint_configs_.push_back(j3);
    prev_joint_configs_.push_back(j4);
    prev_joint_configs_.push_back(j5);
    prev_joint_configs_.push_back(j6);
    // ROS_INFO_STREAM(prev_joint_configs_.size());
    return true;
  } 
  else {
    return false;
  }
}

std::vector<double> JPSPlanner::getToppraPos() const {return toppra_pos_;};

std::vector<double> JPSPlanner::getToppraVel() const {return toppra_vel_;};

std::vector<double> JPSPlanner::getToppraAcc() const {return toppra_acc_;};

std::vector<double> JPSPlanner::getToppraT() const {return toppra_t_;};

bool JPSPlanner::gotoCallback(rokae_jps_navigation::Goto::Request &req, rokae_jps_navigation::Goto::Response &res) 
{
  auto time_start = std::chrono::high_resolution_clock::now();

  // wait for initialization
  if (!is_initialized_) 
  {
    res.message = "Goto rejected, node not initialized.";
    res.success = false;
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Goto rejected, node not initialized." ANSI_COLOR_RESET "\n");

    return false;
  }

  // wait for octomap message
  if (!getting_octomap_) 
  {
    res.message = "Goto rejected, octomap not received.";
    res.success = false;
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Goto rejected, octomap not received." ANSI_COLOR_RESET "\n");

    return false;
  }
  
  // check octomap status
  if (octree_ == NULL || octree_->size() < 1) {
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Octomap is NULL or empty! Abort planning." ANSI_COLOR_RESET "\n");

    return false;
  }


  // initialize parament values
  currGoal_id_ = 0;
  path_.clear();
  raw_path_.clear();
  optimal_path_.clear();
  dist_path_keys_.clear();
  tarPose_buffer_.clear();
  tarPoints_bufferIn.clear();
  tarPoints_pose_.clear();
  tarPoints_pose_msg_.clear();
  joint_configs_.clear();
  path_signer_.clear();
  check_status_ = false;
  setInitialJoints(0, 0, 0, 0, 0, 0);

  printf(ANSI_COLOR_CYAN "[rokae_JPS_planner]: Received Goal, checking..." ANSI_COLOR_RESET "\n");
  
  for (auto &pose : req.goal_pose)
  {
    // store the goal rotation infomation
    Eigen::Vector4f  posture;
    posture[0] = pose.orientation.x;
    posture[1] = pose.orientation.y;
    posture[2] = pose.orientation.z;
    posture[3] = pose.orientation.w;

    // store the goal translation infomation
    octomap::point3d point(pose.position.x, pose.position.y, pose.position.z);

    // subtly change the goals'position to fit the coordinate limitation, moving them to central voxel
    point = suit_coordinate(point);

    // now check the modifed position validity, isExist? isOutSide? isOccupied?
    if (checkAvailability(point)) {
      tarPoints_bufferIn.push_back(point);
      tarPoints_pose_.push_back(posture);

      // store the poses and points information for further validity
      geometry_msgs::Pose modified_pose;
      modified_pose.position.x    = point.x();
      modified_pose.position.y    = point.y();
      modified_pose.position.z    = point.z();
      modified_pose.orientation.x = posture.x();
      modified_pose.orientation.y = posture.y();
      modified_pose.orientation.z = posture.z();
      modified_pose.orientation.w = posture.w();
      tarPose_buffer_.push_back(modified_pose);
    } 
    else {
      std::cout << point << std::endl;
      printf(ANSI_COLOR_RED " goal point is invalid!" ANSI_COLOR_RESET "\n");
      ROS_INFO_STREAM(point);

      return false;
    }
  }
  
  // no goal point received
  if (currGoal_id_ >= (int)tarPoints_bufferIn.size() || tarPoints_bufferIn.empty()) 
  {
    res.message = "Goto rejected, no waypoint provided";
    res.success = false;
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: 'Goto' rejected, providing no waypoint." ANSI_COLOR_RESET "\n");
    
    return false;
  }
  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: All goal positions are valid!" ANSI_COLOR_RESET "\n");

  if(!tarPoseAvalability_client(tarPose_buffer_)) {
    planning_status_ = PlanningState::ERROR;
    printf(ANSI_COLOR_RED "GOAL POSE INVALID!!!" ANSI_COLOR_RESET "\n");

    return false;
  }
  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: All goal poses are valid!" ANSI_COLOR_RESET "\n");


  // receive current end effector pose
  prevPose_ = eef_state_client();
  printf(ANSI_COLOR_CYAN "[rokae_JPS_planner]: Received Start, checking..." ANSI_COLOR_RESET "\n");
  octomap::point3d tmpPoint(prevPose_.position.x, prevPose_.position.y, prevPose_.position.z);
  prevGoal_ = suit_coordinate(tmpPoint);

  // visualize the start point
  int tmp_msg_id = 100;
  visualizePoint(prevGoal_, tmp_msg_id);

  if (!checkAvailability(prevGoal_)) {
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Start point is invalid." ANSI_COLOR_RESET "\n");
    
    return false;
  }  
  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Start point is valid!" ANSI_COLOR_RESET "\n");

  std::vector<geometry_msgs::Pose> prevPoseSet;
  prevPose_.position.x = prevGoal_.x();
  prevPose_.position.y = prevGoal_.y();
  prevPose_.position.z = prevGoal_.z();
  prevPoseSet.push_back(prevPose_);
  if(!tarPoseAvalability_client(prevPoseSet)) {
    planning_status_ = PlanningState::ERROR;
    printf(ANSI_COLOR_RED "START POSE INVALID!!!" ANSI_COLOR_RESET "\n");

    return false;
  }
  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Start pose is valid!" ANSI_COLOR_RESET "\n");


  // begin planning
  printf(ANSI_COLOR_MAGENTA "[rokae_JPS_planner]: Start planning ..." ANSI_COLOR_RESET "\n");

  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Waypoint [%.3f, %.3f, %.3f] set as original point position." ANSI_COLOR_RESET "\n",
                                                                            prevGoal_.x(), prevGoal_.y(), prevGoal_.z());
  printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Pose (x,y,z,w) = [%.3f, %.3f, %.3f, %.3f] set as a original orientation." ANSI_COLOR_RESET "\n",
                            prevPose_.orientation.x, prevPose_.orientation.y, prevPose_.orientation.z, prevPose_.orientation.w);


  // printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: goal buffer size is %d" ANSI_COLOR_RESET "\n", static_cast<int>(tarPoints_bufferIn.size()));
  // printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: pose buffer size is %d" ANSI_COLOR_RESET "\n", static_cast<int>(tarPoints_pose_.size()));

  // clear obstacle iteration id
  obs_iter_id_ = 0;
  std::vector<octomap::point3d> path_points_info;

  // If we send more than one goal we need to plan a path to connect each one in order.
  for (int goalNum = 0; goalNum < (int)tarPoints_bufferIn.size(); goalNum++)
  {
    // time tick for current goal planning
    auto curr_goal_start = std::chrono::high_resolution_clock::now(); 

    // save the initial joint for each goal
    std::vector<float> prev_joints;
    prev_joints.assign(prev_joint_configs_.begin(),prev_joint_configs_.end());

    currGoal_ = tarPoints_bufferIn[currGoal_id_]; 
    
    octomap::point3d planning_start = prevGoal_;
    octomap::point3d planning_goal  = currGoal_;


    printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Waypoint [%.3f, %.3f, %.3f] set as a next goal position." ANSI_COLOR_RESET "\n",
                                                                              planning_goal.x(), planning_goal.y(), planning_goal.z());
    printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Pose (w,x,y,z) = [%.3f, %.3f, %.3f, %.3f] set as a next orientation." ANSI_COLOR_RESET "\n",
                            tarPoints_pose_[currGoal_id_].w(), tarPoints_pose_[currGoal_id_].x(), tarPoints_pose_[currGoal_id_].y(), tarPoints_pose_[currGoal_id_].z());
    printf(ANSI_COLOR_GREEN "[rokae_JPS_planner]: Current goal ID is [%d]\n",currGoal_id_);
    
    visualizePoint(planning_goal, currGoal_id_);

    // initialize the JPS Planner Pointer
    JPS_BasePtr = std::make_shared<JPS::GraphSearch>(xDim_, yDim_, zDim_, planning_tree_resolution_, planning_start, planning_goal, JPS_timeout_, eps_, planner_verbose_);
    JPS_BasePtr->SetHeuristic(heu_type_);  

    planning_status_ = PlanningState::NEED_OPTIMAL;
    while (planning_status_ == PlanningState::NEED_OPTIMAL)
    {
      auto time_now = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration<double>(time_now - time_start).count() > planning_timeout_) 
      {
        printf(ANSI_COLOR_CYAN "[rokae_JPS_planner]: Planning timeout!\n" ANSI_COLOR_RESET);
        std::cout << ANSI_COLOR_BLUE "[rokae_JPS_planner]: Total time cost: " << std::chrono::duration<double>(time_now - time_start).count() << " [s]" ANSI_COLOR_RESET << std::endl;
        
        planning_status_ = PlanningState::ERROR;

        return false;
      }

      prev_joint_configs_.assign(prev_joints.begin(), prev_joints.end());
      bool success = JPS_BasePtr->plan(*octree_, JPS_max_iteration_);
      if (success) {
        // visualize the best optimal path expansion process
        std::vector<JPS::Node> open_set  = JPS_BasePtr->getOpenSet();
        std::vector<JPS::Node> close_set = JPS_BasePtr->getCloseSet();
        visualizeExpansions(open_set, close_set, *octree_);

        // store the original path
        std::vector<octomap::OcTreeKey> path_keys = JPS_BasePtr->getPathKeys(); 
        raw_path_ = keysToCoords(path_keys, *octree_);
        // for (auto &p: raw_path_) {
        //   ROS_INFO("waypoint info: (x,y,z)=(%f,%f,%f)", p.x(), p.y(), p.z());
        // }

        // REMOVE LINE POINTS
        // std::vector<octomap::point3d> rmLPts_Path = removeLinePts(raw_path_, *octree_);

        // TODO THIS FUNCTION NOT PASSED THE TEST
        // iteratively compute the optimal path
        // iterativeComputePath(max_iteration_, *octree_);

        // TODO FOLLOWING CODES USED TO CHECK ORIGINAL JPS PLANNER
        /*******************************************************************/
        path_signer_.clear();
        octomap::KeyRay ray;
        std::vector<octomap::OcTreeKey> keys; 
        for (int i = 0; i < (int)raw_path_.size() - 1; ++i) {
          octree_->computeRayKeys(raw_path_.at(i), raw_path_.at(i+1), ray);
          keys.assign(ray.begin(), ray.end());
          std::vector<octomap::point3d> point  = keysToCoords(keys, *octree_);
          pathNode raw(raw_path_.at(i), true);
          path_signer_.push_back(raw);
          for (size_t j = 1; j < point.size(); j++) {
            pathNode raw(point.at(j), false);
            path_signer_.push_back(raw);
          }
        }
        pathNode raw(raw_path_.back(), true);
        path_signer_.push_back(raw);
        /*******************************************************************/

        // REMOVE CORNER POINTS
        for(int i = 0; i < 3; i++) {
          optimal_path_ = removeCornerPts(path_signer_, *octree_);
        }
        // optimal_path_ = raw_path_;

        // visualize the optimal path
        visualizePath(optimal_path_);
        if (planner_verbose_) {
          printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Stage [1]: JPS Planner success, preparing to validate the path." ANSI_COLOR_RESET "\n");
        }

        // To check if current path is valid, otherwise set the unreachable nodes or the collision nodes as obstacles.
        // The following process will change the 'tree' value.
        tarPoints_pose_msg_.clear();
        resamplePath(optimal_path_, tarPoints_pose_msg_);

        std::vector<std::vector<float>> joint_configs = collision_detection_client(octree_);

        if (planning_status_ == PlanningState::NEED_REPLANNING) {
          // reset the status
          planning_status_ = PlanningState::NEED_OPTIMAL;
          continue;
        } 
        else if (planning_status_ == PlanningState::ERROR) {
          printf(ANSI_COLOR_RED "JPS Planner ERROR happened" ANSI_COLOR_RESET "\n");
          break;
        }

        // current path have no obstacles or collision, 
        // but if TOPP-RA generates a new trajectory, we need to do further check.

        // we need to check the poses of the toppra returned values
        toppra_client(joint_configs,false);
        int8_t joint_num = 0;
        // get joint group
        std::vector<double> joint_val = getToppraPos();
        std::vector<double> joint_group;
        for(auto &jc : joint_val)
        {
          if (joint_num < 6) {
            joint_group.push_back(jc);
            joint_num++;
            if (joint_num == 6) {
              std::vector<geometry_msgs::Pose> pose_group;
              geometry_msgs::Pose pose = joint2pose_client(joint_group);
              octomap::point3d raw_point(pose.position.x, pose.position.y, pose.position.z);
              octomap::point3d suit_point = suit_coordinate(raw_point);
              pose.position.x = suit_point.x();
              pose.position.y = suit_point.y();
              pose.position.z = suit_point.z();
              pose_group.push_back(pose);
              // if status returned is false, this node need to be set as an obstacle
              if (!tarPoseAvalability_client(pose_group)) {
                octomap::OcTreeKey key = octree_->coordToKey(suit_point);
                setCurrNodeObs(key, octree_);
                visualizeVoxel(suit_point);
                // set neighbors obstacles as well
                for (const auto d: JPS_BasePtr->EXPANSION_DIRECTIONS) 
                {
                  auto new_key    = expand(key, d);
                  auto currPoint  = octree_->keyToCoord(new_key);
                  setCurrNodeObs(new_key, octree_);
                  visualizeVoxel(currPoint);
                }
                // need to planning again
                planning_status_ = PlanningState::NEED_REPLANNING;
              }
              joint_group.clear();
              joint_num = 0;
            }
          }
        }
        if (planning_status_ == PlanningState::NEED_REPLANNING) {
          // reset the status
          planning_status_ = PlanningState::NEED_OPTIMAL;
          continue;
        } 
        
        // final joint configs. Now success trajectory for this node generated.
        joint_configs_.insert(joint_configs_.end(), joint_configs.begin(), joint_configs.end());
        // store the path points info and reserve them for plotting.
        path_points_info.insert(path_points_info.end(), optimal_path_.begin(), optimal_path_.end());

        // if planning_status is not NEED_REPLANNING, it must still be the NEED_OPTIMAL, which means the current optimal_path is valid.
        // We set planning state as SUCCESS and move to next planning goal.
        currGoal_id_++;
        prevGoal_        = planning_goal;
        planning_status_ = PlanningState::SUCCESS;
        if (planner_verbose_) {
          printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Stage[2]: Path is valid." ANSI_COLOR_RESET "\n");
        }

        break;
      } 
      else {
        std::cout << ANSI_COLOR_RED "[rokae_JPS_planner]: JPS planning failed, from [start]: (" << planning_start.x() << "," << planning_start.y() << "," << planning_start.z() << ") [goal]: (" << planning_goal.x() << "," << planning_goal.y() << "," << planning_goal.z() << ")" ANSI_COLOR_RESET << " Abort!" << std::endl;
        
        planning_status_ = PlanningState::FAILURE;

        break;
      }
    }

    auto time_cost = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - curr_goal_start);
    std::cout << ANSI_COLOR_BLUE "[rokae_JPS_planner]: JPS Planner time cost for current goal: " << time_cost.count() << " [s]" ANSI_COLOR_RESET << std::endl;

    if (planning_status_ == PlanningState::SUCCESS) {
      printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: [currGoad_id] = [%d] success. Next!" ANSI_COLOR_RESET "\n", currGoal_id_);

      continue;
    }
    else if (planning_status_ == PlanningState::ERROR){
      printf(ANSI_COLOR_RED "[rokae_JPS_planner]: REQUEST FOR RESETING JPS PLANNER" ANSI_COLOR_RESET "\n");

      return false;
    }
    else {
      printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Please check the input pose of start point. [currGoad_id] = [%d]" ANSI_COLOR_RESET "\n", currGoal_id_);

      return false;
    }
  }

  auto time_cost_total = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start);
  std::cout << ANSI_COLOR_BLUE "[rokae_JPS_planner]: Total time cost: " << time_cost_total.count() << " [s]" ANSI_COLOR_RESET << std::endl;

  // using toppra to generate trajectory
  toppra_client(joint_configs_, true);
  printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Genrating the [toppra] trajectory." ANSI_COLOR_RESET "\n");
  // send trajectory data
  res.pos = getToppraPos();
  res.vel = getToppraVel();
  res.acc = getToppraAcc();
  res.t   = getToppraT();

  if (req.ifback) {
    std::reverse(joint_configs_.begin(), joint_configs_.end());
    toppra_client(joint_configs_, true);
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Genrating the [toppra] moving back trajectory." ANSI_COLOR_RESET "\n");

    // send trajectory data
    res.back_pos = getToppraPos();
    res.back_vel = getToppraVel();
    res.back_acc = getToppraAcc();
    res.back_t   = getToppraT();
  }

  res.message = "Planning finished.";
  res.success = true;
  for (auto& p: path_points_info) {
    res.px.push_back(p.x());
    res.py.push_back(p.y());
    res.pz.push_back(p.z());
  }

  return true;

}

DynamicEDTOctomap JPSPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree) 
{
  octomap::point3d metric_min(-xDim_, -yDim_, 0.0);
  octomap::point3d metric_max(xDim_, yDim_, zDim_);
  //- the first argument is the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  DynamicEDTOctomap edf(euclidean_distance_cutoff_, tree.get(), metric_min, metric_max, true);

  //This computes the distance map
  edf.update();

  return edf;
}

void JPSPlanner::octomapCallback(const octomap_msgs::Octomap &msg) 
{
  parent_frame_    = msg.header.frame_id;
  printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Getting octomap" ANSI_COLOR_RESET "\n");

  octomap::AbstractOcTree* treePtr = octomap_msgs::msgToMap(msg);

  // ros::Duration(2);

  if (!treePtr) {
    printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Octomap message is empty!" ANSI_COLOR_RESET "\n");
  } 
  else {
    binary_tree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(treePtr));
  }

  printf(ANSI_COLOR_CYAN "[rokae_JPS_planner]: DynamicEDTOctomap generating ... " ANSI_COLOR_RESET "\n");

  // mtx_.lock();
  auto edf = euclideanDistanceTransform(binary_tree);
  octree_ = std::make_shared<octomap::OcTree>(planning_tree_resolution_);
  binary_tree->expand();
  for (auto it = binary_tree->begin(); it != binary_tree->end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= default_object_padding_) {
      octree_->setNodeValue(it.getCoordinate(), binary_tree->getClampingThresMaxLog()); // obstacle or close to obstacle
    } else {
      octree_->setNodeValue(it.getCoordinate(), binary_tree->getClampingThresMinLog()); // free and safe
    }
  }
  // mtx_.unlock();

  printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: DynamicEDTOctomap generated." ANSI_COLOR_RESET "\n");

  // turn off the octomap subscriber
  if (octree_ != nullptr) {
    octomap_subscriber_.shutdown();
  }

  getting_octomap_ = true;
}

geometry_msgs::Pose JPSPlanner::eef_state_client()
{
  const std::string EEF_CLINET_STR = "/rokae_arm/rokae_eef_state";
  ros::service::waitForService(EEF_CLINET_STR);
  eef_state_client_ = nh_.serviceClient<rokae_jps_navigation::eefState>(EEF_CLINET_STR);

  rokae_jps_navigation::eefState srv;
  srv.request.ifVerbose = planner_verbose_; 

  if (eef_state_client_.call(srv))
  {
    if (planner_verbose_) {
     printf(ANSI_COLOR_MAGENTA "[rokae_eef_state]: end_effector position (x,y,z) = (%.3f,%.3f,%.3f)" ANSI_COLOR_RESET "\n", 
                                                srv.response.eef_pose.position.x, srv.response.eef_pose.position.y, srv.response.eef_pose.position.z);
    }

    if (planner_verbose_) {
     printf(ANSI_COLOR_MAGENTA "[rokae_eef_state]: end_effector orientation (w,x,y,z) = (%.3f,%.3f,%.3f,%.3f)" ANSI_COLOR_RESET "\n", 
                                                srv.response.eef_pose.orientation.w, srv.response.eef_pose.orientation.x, srv.response.eef_pose.orientation.y, srv.response.eef_pose.orientation.z);
    }

    return srv.response.eef_pose;
  }
  else
  {
    printf(ANSI_COLOR_RED "[rokae_eef_state]: client receive no data." ANSI_COLOR_RESET "\n");
    return geometry_msgs::Pose();
  }
}

/* collision detection or state detection client */
std::vector<std::vector<float>> JPSPlanner::collision_detection_client(std::shared_ptr<octomap::OcTree> &tree)
{ 
  if (debug_verbose_) {
    printf(ANSI_COLOR_MAGENTA "[rokae_collision_detection]: collision detection client on" ANSI_COLOR_RESET "\n");
  }
  check_status_ = true;
  const std::string COLLISION_DETECTION_CLIENT_STR = "/rokae_arm/robot_collision_detection_node";

  ros::service::waitForService(COLLISION_DETECTION_CLIENT_STR);
  collision_detection_client_ = nh_.serviceClient<rokae_jps_navigation::CheckCollision>(COLLISION_DETECTION_CLIENT_STR);

  std::vector<std::vector<float>> joint_configs;
  joint_configs.push_back(prev_joint_configs_);

  // ROS_INFO("original joint configs size: %d", (int)joint_configs.size());
  // ROS_INFO("target point pose messages size: %d",(int)tarPoints_pose_msg_.size());
  
  for (auto &pose_msg: tarPoints_pose_msg_)
  {
    // ROS_INFO("(x,y,z)=(%f,%f,%f); (w,i,j,k)=(%f,%f,%f,%f) ",pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z );
    rokae_jps_navigation::CheckCollision collision_detection_srv;
    collision_detection_srv.request.path_pose = pose_msg;
    collision_detection_srv.request.ifVerbose = debug_verbose_;
    collision_detection_srv.request.prev_joints.assign(prev_joint_configs_.begin(), prev_joint_configs_.end());

    octomap::point3d point (pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    octomap::OcTreeKey currNodeKey = tree->coordToKey(point);
    // ROS_INFO("curr_joints size is %d", (int)curr_joint_configs_.size());
    // ROS_INFO("prev_joints size is %d", (int)collision_detection_srv.request.prev_joints.size());
    // printf(ANSI_COLOR_MAGENTA "previous joint configs=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f); iteration=%d" ANSI_COLOR_RESET "\n",prev_joint_configs_.at(0), prev_joint_configs_.at(1), prev_joint_configs_.at(2), prev_joint_configs_.at(3), prev_joint_configs_.at(4), prev_joint_configs_.at(5), iteration);

    // if (isOccupied(tree->search(currNodeKey))) {
    //   printf(ANSI_COLOR_RED "CHECK STATUS OCCUPIED" ANSI_COLOR_RESET "\n");
    //   planning_status_ = PlanningState::ERROR;
    //   return;
    // }

    if (collision_detection_client_.call(collision_detection_srv))
    {
      if(collision_detection_srv.response.getSolution) {
        if(collision_detection_srv.response.isCollide) {
          if (debug_verbose_) {
            printf(ANSI_COLOR_RED "manipulator collision" ANSI_COLOR_RESET "\n");
          }
          setCurrNodeObs(currNodeKey, tree);
          visualizeVoxel(tree->keyToCoord(currNodeKey));
          obs_iter_id_++;
          planning_status_ = PlanningState::NEED_REPLANNING;
        } 
        else {
          curr_joint_configs_.assign(collision_detection_srv.response.curr_joints.begin(), collision_detection_srv.response.curr_joints.end());
          prev_joint_configs_.assign(curr_joint_configs_.begin(), curr_joint_configs_.end());
          if (debug_verbose_) {
            printf(ANSI_COLOR_GREEN "Current pose safe. Get joint configs." ANSI_COLOR_RESET "\n");
          }
          joint_configs.push_back(curr_joint_configs_);
        }
      }
      else {
        if (debug_verbose_) {
          printf(ANSI_COLOR_RED "violated state" ANSI_COLOR_RESET "\n");
        }
        setCurrNodeObs(currNodeKey, tree);
        visualizeVoxel(tree->keyToCoord(currNodeKey));
        obs_iter_id_++;
        // violated_iter++;
        planning_status_ = PlanningState::NEED_REPLANNING;
      }
    }
  }

  if (planning_status_ != PlanningState::NEED_REPLANNING) {
    // joint_configs_.insert(joint_configs_.end(), joint_configs.begin(), joint_configs.end());
    printf(ANSI_COLOR_MAGENTA "[rokae_collision_detection]: Get effective joint configs. SIZE %d" ANSI_COLOR_RESET "\n", static_cast<int>(joint_configs.size()));
    return joint_configs;
  } 
  else {
    printf(ANSI_COLOR_RED "Current path contains some obstacles or unreachable points. NEED_REPLANNING" ANSI_COLOR_RESET "\n");
    
    return std::vector<std::vector<float>>();
  }
}


bool JPSPlanner::tarPoseAvalability_client(std::vector<geometry_msgs::Pose> &tarPoseSet)
{
  if (debug_verbose_) {
    printf(ANSI_COLOR_MAGENTA "[tarPoseAvalability_client]: target poses avalability checker client on" ANSI_COLOR_RESET "\n");
  }

  // using collision detection client handle
  const std::string COLLISION_DETECTION_CLIENT_STR = "/rokae_arm/robot_collision_detection_node";
  ros::service::waitForService(COLLISION_DETECTION_CLIENT_STR);
  collision_detection_client_ = nh_.serviceClient<rokae_jps_navigation::CheckCollision>(COLLISION_DETECTION_CLIENT_STR);

  int  iteration = 0;
  bool status    = true;
  for (auto &pose_msg: tarPoseSet) {
    rokae_jps_navigation::CheckCollision collision_detection_srv;
    // ROS_INFO("tarpose: (x,y,z,w,i,j,k): (%f,%f,%f,%f,%f,%f,%f)", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
    collision_detection_srv.request.path_pose = pose_msg;
    collision_detection_srv.request.ifVerbose = debug_verbose_;
    collision_detection_srv.request.prev_joints.assign(prev_joint_configs_.begin(), prev_joint_configs_.end());

    iteration++;
    if (collision_detection_client_.call(collision_detection_srv)) {
      if(collision_detection_srv.response.getSolution) {
        if(collision_detection_srv.response.isCollide) {
          if (debug_verbose_) {
            printf(ANSI_COLOR_RED "target pose [%d] is in collision, please check the pose set." ANSI_COLOR_RESET "\n", iteration);
          }
          status = false;
        }
        else {
          if (debug_verbose_) {
            printf(ANSI_COLOR_GREEN "target pose [%d] is valid" ANSI_COLOR_RESET "\n", iteration);
          }
        }
      } 
      else {
        if (debug_verbose_) {
          printf(ANSI_COLOR_RED "target pose [%d] is invalid, please check the pose set." ANSI_COLOR_RESET "\n", iteration);
        }
        status = false;
      }
    }
  }

  return status;
}

geometry_msgs::Pose JPSPlanner::joint2pose_client(std::vector<double> &joint_group)
{
  if (debug_verbose_) {
    printf(ANSI_COLOR_MAGENTA "[joint2pose_client]: joints to poses convertor client on" ANSI_COLOR_RESET "\n");
  }

  geometry_msgs::Pose pose;
  // using joint2pose client handle
  const std::string JOINT2POSE_CLIENT_STR = "/rokae_arm/rokae_joint2pose";
  ros::service::waitForService(JOINT2POSE_CLIENT_STR);
  joint2pose_client_ = nh_.serviceClient<rokae_jps_navigation::joint2pose>(JOINT2POSE_CLIENT_STR);
  rokae_jps_navigation::joint2pose joint2pose_srv;
  joint2pose_srv.request.ifVerbose = false;
  joint2pose_srv.request.joint0 = joint_group.at(0);
  joint2pose_srv.request.joint1 = joint_group.at(1);
  joint2pose_srv.request.joint2 = joint_group.at(2);
  joint2pose_srv.request.joint3 = joint_group.at(3);
  joint2pose_srv.request.joint4 = joint_group.at(4);
  joint2pose_srv.request.joint5 = joint_group.at(5);
  if (joint2pose_client_.call(joint2pose_srv)) {
    pose = joint2pose_srv.response.re_pose;
  } else {
    printf(ANSI_COLOR_RED "[joint2pose_client]: ERROR." ANSI_COLOR_RESET "\n");
  }

  return pose;
}

void JPSPlanner::resamplePath(std::vector<octomap::point3d> &waypoints, std::vector<geometry_msgs::Pose> &tarPoints_pose_msg_local)
{
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: resample path with pose" ANSI_COLOR_RESET "\n");
  }
  geometry_msgs::Pose pose;
  std::vector<geometry_msgs::Pose> pose_msg;

  if (currGoal_id_ == 0) {
    pose_msg.push_back(prevPose_);
  } // currGoal_id_ is 0 means, 
  else {
    pose_msg.push_back(tarPose_buffer_[currGoal_id_-1]);
  }

  int container_size = waypoints.size();
  for (int i = 0; i < container_size; ++i) 
  {
    // if we have a policy to determine the pose, this function need to be changed.
    pose.position.x    = waypoints[i].x();
    pose.position.y    = waypoints[i].y();
    pose.position.z    = waypoints[i].z();
    pose.orientation.x = tarPoints_pose_[currGoal_id_].x();
    pose.orientation.y = tarPoints_pose_[currGoal_id_].y();
    pose.orientation.z = tarPoints_pose_[currGoal_id_].z();
    pose.orientation.w = tarPoints_pose_[currGoal_id_].w();
    pose_msg.push_back(pose);
  }
  tarPoints_pose_msg_local.assign(pose_msg.begin(), pose_msg.end());

  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: local pose message size is %d" ANSI_COLOR_RESET "\n", static_cast<int>(tarPoints_pose_msg_local.size()));
  }

  // we need to enlarge the 'waypoint' set, add the front pose_msg's position infomation
  octomap::point3d front_element(pose_msg.front().position.x, pose_msg.front().position.y, pose_msg.front().position.z);
  waypoints.insert(waypoints.begin(), front_element);
  
}


bool JPSPlanner::toppra_client(std::vector<std::vector<float>> &joint_group, bool ifSave)
{
  // clear previous status
  toppra_pos_.clear();
  toppra_vel_.clear();
  toppra_acc_.clear();
  toppra_t_.clear();

  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: generating trajectory" ANSI_COLOR_RESET "\n");
  }

  std::vector<float> joint_configs_1d;

  for (auto &jc : joint_group) {
    for(size_t i = 0; i < jc.size(); i++) {
      joint_configs_1d.push_back(jc.at(i));
    }
  }

  const std::string TOPPRA_CLIENT_STR = "/rokae_arm/toppra";
  ros::service::waitForService(TOPPRA_CLIENT_STR);
  toppra_client_ = nh_.serviceClient<rokae_arm_toppra::ToppRa_srv>(TOPPRA_CLIENT_STR);

  rokae_arm_toppra::ToppRa_srv toppra_srv;
  toppra_srv.request.ifSave = ifSave;
  toppra_srv.request.joint_configs_on_way.assign(joint_configs_1d.begin(), joint_configs_1d.end());

  if (toppra_client_.call(toppra_srv))
  {
    for (auto &pos:toppra_srv.response.pos) {
      toppra_pos_.push_back(pos);
    }
    for (auto &vel:toppra_srv.response.vel) {
      toppra_vel_.push_back(vel);
    }
    for (auto &acc:toppra_srv.response.acc) {
      toppra_acc_.push_back(acc);
    }
    for (auto &t:toppra_srv.response.t) {
      toppra_t_.push_back(t);
    }
    if (planner_verbose_) {
      printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: trajectory generated" ANSI_COLOR_RESET "\n");
    }

    return true;
  }
  else 
  {
    if (planner_verbose_) {
      printf(ANSI_COLOR_RED "Calling toppra server failed" ANSI_COLOR_RESET "\n");
    }

    return false;
  }
}

void JPSPlanner::visualizeTree(const octomap::OcTree &tree) 
{
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Visualizing tree" ANSI_COLOR_RESET "\n");
  }
  visualization_msgs::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = ros::Time().now();
  msg.ns                 = "tree";
  msg.type               = visualization_msgs::Marker::POINTS;
  msg.id                 = 100000;
  msg.action             = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = tree_points_scale_;
  msg.scale.y            = tree_points_scale_;
  msg.scale.z            = tree_points_scale_;

  // unsigned long int count = 0;
  for (auto it = tree.begin(); it != tree.end(); it++) 
  {
    if (it->getOccupancy() >= 0.7) {
      geometry_msgs::Point goal_point;
      auto color                      = generateColor(0, 0, 0, 1.0);
      goal_point.x                    = it.getX();
      goal_point.y                    = it.getY();
      goal_point.z                    = it.getZ();
      msg.points.push_back(goal_point);
      msg.colors.push_back(color);
    } 
    else if (show_unoccupied_) 
    {
      geometry_msgs::Point goal_point;
      auto color                      = generateColor(0.7, 0.7, 0.7, 0.7);
      goal_point.x                    = it.getX();
      goal_point.y                    = it.getY();
      goal_point.z                    = it.getZ();
      msg.points.push_back(goal_point);
      msg.colors.push_back(color);
    }
    // count++;
    // ROS_INFO_STREAM(count);
  }
  msg.lifetime = ros::Duration(0);
  binary_tree_publisher_.publish(msg);
}

void JPSPlanner::visualizeExpansions(const std::vector<JPS::Node> &open, const std::vector<JPS::Node> &closed, const octomap::OcTree &tree) 
{
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Visualizing open planning expansions" ANSI_COLOR_RESET "\n");
  }
  visualization_msgs::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = ros::Time().now();
  msg.ns                 = "expansions";
  msg.type               = visualization_msgs::Marker::POINTS;
  msg.id                 = currGoal_id_;
  msg.action             = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = expansions_points_scale_;
  msg.scale.y            = expansions_points_scale_;

  double max_cost = 0;
  double min_cost = 1e6;

  for (auto it = open.begin(); it != open.end(); it++) 
  {
    if (it->f > max_cost) {
      max_cost = it->f;
    }
    if (it->f < min_cost) {
      min_cost = it->f;
    }
  }

  for (auto it = open.begin(); it != open.end(); it++) 
  {
    auto coords                      = tree.keyToCoord(it->key);
    geometry_msgs::Point goal_point;
    double brightness                = (it->f - min_cost) / (max_cost - min_cost);
    auto color                       = generateColor(0.0, brightness, 0.3, 0.8); // the less cost, the brighter 
    goal_point.x                     = coords.x();
    goal_point.y                     = coords.y();
    goal_point.z                     = coords.z();
    msg.points.push_back(goal_point);
    msg.colors.push_back(color);
  }

  for (auto it = closed.begin(); it != closed.end(); it++) 
  {
    auto coords                      = tree.keyToCoord(it->key);
    geometry_msgs::Point goal_point;
    auto color                       = generateColor(0.8, 0.0, 0, 0.8);
    goal_point.x                     = coords.x();
    goal_point.y                     = coords.y();
    goal_point.z                     = coords.z();
    msg.points.push_back(goal_point);
    msg.colors.push_back(color);
  }

  // msg.lifetime = ros::Duration(0);

  expansion_publisher_.publish(msg);
}

void JPSPlanner::visualizePath(const std::vector<octomap::point3d> &waypoints) 
{
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Visualizing path" ANSI_COLOR_RESET "\n");
  }
  visualization_msgs::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = ros::Time().now();
  msg.ns                 = "path";
  msg.type               = visualization_msgs::Marker::LINE_STRIP;
  msg.id                 = currGoal_id_;
  msg.action             = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = path_points_scale_;
  msg.scale.y            = path_points_scale_;
  msg.scale.z            = path_points_scale_;
  
  for (size_t i = 1; i < waypoints.size(); i++) {
    geometry_msgs::Point p1, p2;
    std_msgs::ColorRGBA  c;
    p1.x = waypoints[i - 1].x();
    p1.y = waypoints[i - 1].y();
    p1.z = waypoints[i - 1].z();
    p2.x = waypoints[i].x();
    p2.y = waypoints[i].y();
    p2.z = waypoints[i].z();
    c    = generateColor(0.1 * currGoal_id_ , double(i) / double(waypoints.size()), 0.1, 1);
    msg.points.push_back(p1);
    msg.points.push_back(p2);
    msg.colors.push_back(c);
    msg.colors.push_back(c);
  }
  path_publisher_.publish(msg);
}

void JPSPlanner::visualizePoint(const octomap::point3d &point, int msg_id) 
{
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Visualizing goals" ANSI_COLOR_RESET "\n");
  }
  visualization_msgs::Marker msg;
  msg.header.frame_id    = "/world";
  msg.header.stamp       = ros::Time().now();
  msg.ns                 = "point";
  msg.type               = visualization_msgs::Marker::SPHERE_LIST;
  msg.id                 = msg_id;
  msg.action             = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = point_scale_;
  msg.scale.y            = point_scale_;
  msg.scale.z            = point_scale_;

  std_msgs::ColorRGBA c = generateColor(0.3, 0.5, 1.0, 1.0);
  geometry_msgs::Point msg_point;

  msg_point.x = point.x();
  msg_point.y = point.y();
  msg_point.z = point.z();

  msg.points.push_back(msg_point);
  msg.colors.push_back(c);
  point_publisher_.publish(msg);
}

void JPSPlanner::visualizeVoxel(const octomap::point3d &point) 
{
  if (debug_verbose_) {
    printf(ANSI_COLOR_BLUE "[rokae_JPS_planner]: Visualizing voxel" ANSI_COLOR_RESET "\n");
  }
  visualization_msgs::Marker msg;
  msg.header.frame_id    = "/world";
  msg.header.stamp       = ros::Time().now();
  msg.ns                 = "obs";
  msg.type               = visualization_msgs::Marker::CUBE;
  msg.id                 = 20+obs_iter_id_;
  msg.action             = visualization_msgs::Marker::ADD;
  msg.pose.position.x    = point.x();
  msg.pose.position.y    = point.y();
  msg.pose.position.z    = point.z();
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = voxel_scale_;
  msg.scale.y            = voxel_scale_;
  msg.scale.z            = voxel_scale_;
  msg.lifetime           = ros::Duration();
  msg.color.r            = 1.0;
  msg.color.g            = 0.0;
  msg.color.b            = 0.5;
  msg.color.a            = 1.0;

  voxel_publisher_.publish(msg);
}

std_msgs::ColorRGBA JPSPlanner::generateColor(const double r, const double g, const double b, const double a) 
{
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

bool JPSPlanner::plan(octomap::OcTree &tree)
{
  path_.clear();

  path_cost_ = JPS_BasePtr->DPF_Plan(tree, region_set, dist_map, potential_weight_);

  std::vector<octomap::OcTreeKey> keys = JPS_BasePtr->getPathKeys();
  if (keys.size() < 1) {
    if(planner_verbose_) {
      std::cout << ANSI_COLOR_RED "Cannot find a path in interatively compute progress. Abort!" ANSI_COLOR_RESET << std::endl;
      return false;
    }
  }

  path_ = keysToCoords(keys, tree); 
  // Simplify the raw path

  return true;
}


bool JPSPlanner::iterativeComputePath(int max_iteration, octomap::OcTree &tree)
{
  if (planner_verbose_)
  {
    printf("\n" ANSI_COLOR_BLUE "*********** [Iterative Compute Optimal Path] ***********" ANSI_COLOR_RESET"\n");
    printf(ANSI_COLOR_BLUE "eps: %f" ANSI_COLOR_RESET"\n", eps_);
    printf(ANSI_COLOR_BLUE "potential weight: %f" ANSI_COLOR_RESET"\n", potential_weight_);
    printf(ANSI_COLOR_BLUE "potential radius: %d" ANSI_COLOR_RESET"\n", potential_radius_);
    printf(ANSI_COLOR_BLUE "search region radius: %d" ANSI_COLOR_RESET"\n", search_radius_);
    printf(ANSI_COLOR_BLUE "max iteration time: %d" ANSI_COLOR_RESET"\n", max_iteration);
  }

  // Simplify the raw path

  optimal_path_ = raw_path_;

  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[IterativeDPFlanner]: input path size: %d" ANSI_COLOR_RESET "\n", static_cast<int>(optimal_path_.size()));
  }

  std::vector<octomap::OcTreeKey> optimal_keys = coordsToKeys(optimal_path_, tree);
  double prev_cost = std::numeric_limits<double>::infinity();

  

  for (int i = 0; i < max_iteration; ++i)
  {
    setSearchRegion(optimal_keys, search_radius_, potential_radius_, tree);
    if (!plan(tree)) {
      return false;
    }

    if (path_cost_ == prev_cost) {
      if (planner_verbose_) {
        printf(ANSI_COLOR_BLUE "[IterativeDPFlanner]: converged to %f!" ANSI_COLOR_RESET "\n", prev_cost);
      }
      break;
    } 
    else {
      prev_cost     = path_cost_;
      optimal_path_ = path_;
      if (planner_verbose_) {
        printf(ANSI_COLOR_BLUE "[IterativeDPFlanner]: path cost is %f! iteration: %d" ANSI_COLOR_RESET "\n", prev_cost, i);
      }
    }
  }

  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[IterativeDPFlanner]: Optimized path size is %d" ANSI_COLOR_RESET "\n", static_cast<int>(optimal_path_.size()));
  }

  return true;
}

void JPSPlanner::setSearchRegion(std::vector<octomap::point3d> &path, const int search_radius, const int potential_radius, octomap::OcTree &tree)
{
  // if radius is negative or the search_radius is bigger than potential_radius, there will exist no tunnel
  if (search_radius <= 0 || potential_radius <=0 || search_radius >= potential_radius) {
    return;
  }
  
  // clear previous value
  region_set.clear();
  dist_map.clear();
  dist_path_keys_.clear();

  octomap::KeyRay ray;

  for (int i = 0; i < (int)path.size() - 1; ++i) {
    tree.computeRayKeys(path.at(i), path.at(i+1), ray);
    dist_path_keys_.insert(dist_path_keys_.end(), ray.begin(), ray.end());
  }
  dist_path_keys_.push_back(tree.coordToKey(path.back()));

  if (planner_verbose_) {
    printf(ANSI_COLOR_MAGENTA "[IterativeDPFlanner]: distance path keys size: %d" ANSI_COLOR_RESET "\n", static_cast<int>(dist_path_keys_.size()));
  }
  // get tunnel region neighbors'increment set
  std::vector<std::vector<int>> SEARCH_REGION_NEIGHBORS_SET;
  getDirSet(search_radius, SEARCH_REGION_NEIGHBORS_SET);

  // get obstacle neighbors'increment set for distance potential field map building
  std::vector<std::vector<int>> OBS_DPF_NEIGHBORS_SET;
  getDirSet(potential_radius, OBS_DPF_NEIGHBORS_SET);

  // store OcTreeKeys of search region in region_set
  // If the planner finds a node is in search region and this node is occupied, we need to calculate the distance potential field value around this node.
  for (auto &key: dist_path_keys_) {
    for (auto &sd: SEARCH_REGION_NEIGHBORS_SET) {
      octomap::OcTreeKey new_key   = expand(key, sd);
      octomap::point3d   new_coord = tree.keyToCoord(new_key);
      // if the new_coord violate the edge limitation, planner will continue to process the next node and will not add this node to the region_set.
      if (isOutside(new_coord)) {
        continue;
      }
      // if the region_set has not contained the 'new_key', that means the 'new_key' is a search region node we need to consider.
      auto find_result = region_set.find(new_key);
      if (find_result == region_set.end()) {
        region_set.insert(new_key);
        octomap::OcTreeNode* newNodePtr = tree.search(new_key);
        // if the new node is occupied, we need to do DPF process
        if (isOccupied(newNodePtr)) {
          dist_map[new_key] = mask_gain_; // current 'new_key' node is occupied, so we assigned 'mask_gain_' value to it.
          for (auto &od: OBS_DPF_NEIGHBORS_SET) {
            octomap::OcTreeKey new_newKey   = expand(new_key, od);
            octomap::point3d   new_newCoord = tree.keyToCoord(new_newKey);
            // if the neighbor around the occupied node is outside of the whole map limitation, we abandon this node to process the next neighbor node
            if (isOutside(new_newCoord)) {
              continue;
            }
            double dist_ratio = 1.0 - (double)(std::abs(od[0]) + std::abs(od[1]) + std::abs(od[2])) / (3*potential_radius);
            double mask_val   = mask_gain_ * (dist_ratio * dist_ratio);
            if (dist_map.find(new_newKey) == dist_map.end()) { // new in 'dist_map'
              dist_map[new_newKey] = mask_val;
            } else {
              dist_map[new_newKey] = std::max(dist_map[new_newKey], mask_val);
            }
          }
        }
      }
    }
  }
}

void JPSPlanner::setSearchRegion(std::vector<octomap::OcTreeKey> &keys, const int search_radius, const int potential_radius, octomap::OcTree &tree)
{
  // if radius is negative or the search_radius is bigger than potential_radius, there will exist no tunnel
  if (search_radius <= 0 || potential_radius <=0) {
    return;
  }

  // clear previous value
  region_set.clear();
  dist_map.clear();
  dist_path_keys_.clear();
  
  std::vector<octomap::point3d> coords = keysToCoords(keys, tree);
  octomap::KeyRay ray;

  for (int i = 0; i < (int)coords.size() - 1; ++i) {
    tree.computeRayKeys(coords.at(i), coords.at(i+1), ray);
    dist_path_keys_.insert(dist_path_keys_.end(), ray.begin(), ray.end());
  }
  dist_path_keys_.push_back(keys.back());

  if (planner_verbose_) {
    printf(ANSI_COLOR_MAGENTA "[IterativeDPFlanner]: distance path keys size: %d" ANSI_COLOR_RESET "\n", static_cast<int>(dist_path_keys_.size()));
  }

  // get tunnel region neighbors'increment set
  std::vector<std::vector<int>> SEARCH_REGION_NEIGHBORS_SET;
  getDirSet(search_radius, SEARCH_REGION_NEIGHBORS_SET);

  // get obstacle neighbors'increment set for distance potential field map building
  std::vector<std::vector<int>> OBS_DPF_NEIGHBORS_SET;
  getDirSet(potential_radius, OBS_DPF_NEIGHBORS_SET);

  // store OcTreeKeys of search region in region_set
  // If the planner finds a node is in search region and this node is occupied, we need to calculate the distance potential field value around this node.
  for (auto &key: dist_path_keys_) {
    for (auto &sd: SEARCH_REGION_NEIGHBORS_SET) {
      octomap::OcTreeKey new_key   = expand(key, sd);
      octomap::point3d   new_coord =tree.keyToCoord(new_key);
      // if the new_coord violate the edge limitation, planner will continue to process the next node and will not add this node to the region_set.
      if (isOutside(new_coord)) {
        continue;
      }
      // if the region_set has not contained the 'new_key', that means the 'new_key' is a search region node we need to consider.
      auto find_result = region_set.find(new_key);
      if (find_result == region_set.end()) {
        region_set.insert(new_key);
        octomap::OcTreeNode* newNodePtr = tree.search(new_key);
        // if the new node is occupied, we need to do DPF process
        if (isOccupied(newNodePtr)) {
          dist_map[new_key] = mask_gain_; // current 'new_key' node is occupied, so we assigned 'mask_gain_' value to it.
          for (auto &od: OBS_DPF_NEIGHBORS_SET) {
            octomap::OcTreeKey new_newKey   = expand(new_key, od);
            octomap::point3d   new_newCoord = tree.keyToCoord(new_newKey);
            // if the neighbor around the occupied node is outside of the whole map limitation, we abandon this node to process the next neighbor node
            if (isOutside(new_newCoord)) {
              continue;
            }
            double dist_ratio = 1.0 - (double)(std::abs(od[0]) + std::abs(od[1]) + std::abs(od[2])) / (3*potential_radius);
            double mask_val   = mask_gain_ * (dist_ratio * dist_ratio);
            if (dist_map.find(new_newKey) == dist_map.end()) { // new in 'dist_map'
              dist_map[new_newKey] = mask_val;
            } else {
              dist_map[new_newKey] = std::max(dist_map[new_newKey], mask_val);
            }
          }
        }
      }
    }
  }
}

void JPSPlanner::getDirSet(const int radius, std::vector<std::vector<int>> &expension_dirs)
{
  std::vector<int> dir;
  for (int nx = -radius; nx <= radius; ++nx) {
    for (int ny = -radius; ny <= radius; ++ny) {
      for (int nz = -radius; nz <= radius; ++nz) {
        dir.push_back(nx);
        dir.push_back(ny);
        dir.push_back(nz);
        expension_dirs.push_back(std::move(dir));
      }
    }
  }
}


octomap::OcTreeKey JPSPlanner::expand(const octomap::OcTreeKey &key, const std::vector<int> &direction) 
{
  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  return k;
}

std::vector<octomap::point3d> JPSPlanner::getPath() const {return path_;};

std::vector<octomap::point3d> JPSPlanner::getRawPath() const {return raw_path_;};

PlanningState JPSPlanner::getPlanningStatus() const {return planning_status_;};

std::vector<octomap::point3d> JPSPlanner::removeLinePts(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree)
{

  if (waypoints.size() < 3) {
    if (planner_verbose_) {
      printf(ANSI_COLOR_BLUE "[JPS-OP]: No enough points for 'Line Points' filtering!" ANSI_COLOR_RESET "\n");
    }
    
    return waypoints;
  }

  /* removing obsolete points */
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[JPS-OP]: Original waypoints size: %d" ANSI_COLOR_RESET "\n", static_cast<int>(waypoints.size()));
  }

  std::vector<octomap::point3d> optimized_path;

  /* front(): this function returns a direct reference */  
  optimized_path.push_back(waypoints.front()); 
  size_t k = 2; // 'front' and 'back' 

  while (k < waypoints.size()) 
  {
    if (!freeStraightPath(optimized_path.back(), waypoints[k], tree)) {
      optimized_path.push_back(waypoints[k - 1]);
    }

    k++;
  }
  optimized_path.push_back(waypoints.back());

  /* removing obsolete points */
  if (planner_verbose_) {
    printf(ANSI_COLOR_BLUE "[JPS-OP]: Optimized waypoints size: %d" ANSI_COLOR_RESET "\n", static_cast<int>(optimized_path.size()));
  }

  // reverse order
  return optimized_path;
}

std::vector<octomap::point3d> JPSPlanner::removeCornerPts(const std::vector<pathNode> &pathNodes, octomap::OcTree &tree)
{
  if (pathNodes.size() < 3) {
    if (planner_verbose_) {
      printf(ANSI_COLOR_BLUE "[JPS]: No enough points for 'Corner Points' filtering!" ANSI_COLOR_RESET "\n");
    }
    
    return raw_path_;
  }

  // cut zigzag segment
  std::vector<octomap::point3d> optimized_path;
  octomap::point3d point1;
  octomap::point3d point2;
  octomap::point3d point3;

  double cost1, cost2, cost3;

  optimized_path.push_back(pathNodes.front().point);
  int iteration = 1;
  while(iteration + 2 < (int) pathNodes.size()) {
    point1 = pathNodes.at(iteration).point;
    point2 = pathNodes.at(iteration+1).point;
    point3 = pathNodes.at(iteration+2).point;
    // calculate cost1
    if (freeStraightPath(point1, point2, tree)) {
      cost1 = (point1 - point2).norm();
    } else {
      cost1 = std::numeric_limits<double>::infinity();
    }
    // calculate cost2
    if (freeStraightPath(point2, point3, tree)) {
      cost2 = (point2 - point3).norm();
    } else {
      cost2 = std::numeric_limits<double>::infinity();
    }
    // calculate cost3
    if (freeStraightPath(point1, point3, tree)) {
      cost3 = (point1 - point3).norm();
    } else {
      cost3 = std::numeric_limits<double>::infinity();
    }

    if (cost3 < cost1 + cost2) {
      optimized_path.push_back(point1);
      // don't delete the raw path points
      if(pathNodes.at(iteration+1).ifRaw) {
        optimized_path.push_back(point2);
      }
    } 
    else if (cost3 == std::numeric_limits<double>::infinity()) {
      iteration += 2;
      continue;
    } // This is because the waypoints is generated from the computeRayKeys() function, so we can just check cost3 value to see if there exists an obstacle between point1 and point3. (We have padding the obstacles and the robot links, generally the code above just waste time)
    else {
      optimized_path.push_back(point1);
      optimized_path.push_back(point2);
    }
    iteration += 2;
  }
  optimized_path.push_back(pathNodes.back().point);

  return optimized_path;
}

std::vector<octomap::point3d> JPSPlanner::keysToCoords(std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree) 
{
  std::vector<octomap::point3d> coords;  
  for (auto &k : keys) {
    octomap::point3d temp_point = tree.keyToCoord(k);
    coords.push_back(temp_point);
  }

  return coords;
}

std::vector<octomap::OcTreeKey> JPSPlanner::coordsToKeys(std::vector<octomap::point3d> &path, octomap::OcTree &tree) 
{
  std::vector<octomap::OcTreeKey> keys;

  for (auto &p : path) {
    octomap::OcTreeKey temp_key = tree.coordToKey(p);
    keys.push_back(temp_key);
  }

  return keys;
}

// octomap::point3d JPSPlanner::keyToCoord_modified(octomap::OcTreeKey &key, octomap::OcTree &tree)
// {
//   float modified_x, modified_y, modified_z;
//   float compenstate_offset    = planning_tree_resolution_/2;

//   octomap::point3d temp_point = tree.keyToCoord(key);
//   if (temp_point.x() < 0) {
//     modified_x = temp_point.x() + compenstate_offset;
//   } else {
//     modified_x = temp_point.x() - compenstate_offset;
//   }

//   if (temp_point.y() < 0) {
//     modified_y = temp_point.y() + compenstate_offset;
//   } else {
//     modified_y = temp_point.y() - compenstate_offset;
//   }

//   if (temp_point.z() < 0) {
//     modified_z = temp_point.z() + compenstate_offset;
//   } else {
//     modified_z = temp_point.z() - compenstate_offset;
//   }

//   octomap::point3d modified_point (modified_x, modified_y, modified_z);

//   return modified_point;
// }


bool JPSPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree)
{
  // Traces a ray from origin to end (excluding), returning the coordinates of all nodes traversed by the beam.
  octomap::KeyRay ray;
  tree.computeRayKeys(p1, p2, ray); // (origin, end, <vector> Ray)

  // ROS_INFO_STREAM("Ray size: " << ray.size());

  for (auto &k : ray) {

    auto tree_node = tree.search(k);

    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      // ROS_WARN_STREAM("Unknown cell");
      return false;
    }

    if (isOccupied(tree_node)) {
      // Path goes through occupied cells
      // ROS_WARN_STREAM("Through occupied cell");
      return false;
    }
  }

  return true;
}

// TODO cannot set obstacles
bool JPSPlanner::setCurrNodeObs(octomap::OcTreeKey &key, std::shared_ptr<octomap::OcTree> &tree) {
  // auto reNode = tree->search(key);
  // reNode->setLogOdds(0.0); // Node's LogOdds >= 0.7 is regard as an obstacle
  auto reNode = tree->search(key);
  if (reNode) {
    // if (planner_verbose_) {
    //   printf(ANSI_COLOR_GREEN "Successfully updated node occupancy value!" ANSI_COLOR_RESET "\n");
    // }
    // ROS_INFO("occupancy value0: %f",reNode->getOccupancy());
    // ROS_INFO("Current Node State0: %s", isOccupied(reNode)?"Occupied":"Free"); 
    tree->setNodeValue(key, tree->getClampingThresMaxLog());
    // reNode->setLogOdds(tree->getClampingThresMaxLog());
    // tree->updateInnerOccupancy();
    // ROS_INFO("occupancy value1: %f",reNode->getOccupancy());
    // ROS_INFO("Current Node State1: %s", isOccupied(reNode)?"Occupied":"Free"); 
    return true;
  } 
  else {
    if (planner_verbose_) {
      printf(ANSI_COLOR_RED "Failed to update node occupancy value! Unknown Node." ANSI_COLOR_RESET "\n");
    }

    return false;
  }
}

bool JPSPlanner::isOccupied(octomap::OcTreeNode* currNode) {
  // default occupancy probability threshold is set to 0.7
  // the `value` store the log-odds value, so getOccupancy run probability() function to do type transfer

  // TODO isOccupied
  // ROS_INFO("current Node Occupancy: %f", currNode->getOccupancy());
  return currNode!=NULL && currNode->getOccupancy() > 0.5;
  // return currNode!=NULL && !octree_->isNodeOccupied(currNode);
}

bool JPSPlanner::isFree(octomap::OcTreeNode* currNode) {
  return currNode!=NULL && currNode->getOccupancy() <= 0.5;
  // return currNode!=NULL && octree_->isNodeOccupied(currNode);
}

bool JPSPlanner::isOutside(const octomap::point3d point)  {
  return (point.x() <= -xDim_ || point.x() >= xDim_ || 
          point.y() <= -yDim_ || point.y() >= yDim_ || point.z() < 0 || point.z() >= zDim_);
}

octomap::point3d JPSPlanner::suit_coordinate(octomap::point3d &point) 
{
  float x, y, z;
  int integral_x = (int)(point.x()/planning_tree_resolution_);
  int integral_y = (int)(point.y()/planning_tree_resolution_);
  int integral_z = (int)(point.z()/planning_tree_resolution_);
  // printf("x, y, z: %d, %d, %d", integral_x, integral_y, integral_z);

  // ROS_INFO_STREAM("integral_x: " << integral_x);
  // ROS_INFO_STREAM("integral_z: " << integral_z);

  if ( std::fabs(point.x() - integral_x * planning_tree_resolution_) >= planning_tree_resolution_ / 2) {
    if (integral_x > 0) {
      x = (integral_x + 0.5) * planning_tree_resolution_;
    } else if (integral_x < 0) {
      x = (integral_x - 0.5) * planning_tree_resolution_;
    } else {
      x = point.x();
    }
  } else {
    if (integral_x > 0) {
      x = (integral_x - 0.5) * planning_tree_resolution_;
    } else if (integral_x < 0) {
      x = (integral_x + 0.5) * planning_tree_resolution_;
    } else {
      x = point.x();
    }
  }

  if ( std::fabs(point.y() - integral_y * planning_tree_resolution_) >= planning_tree_resolution_ / 2) {
    if (integral_y > 0) {
      y = (integral_y + 0.5) * planning_tree_resolution_;
    } else if (integral_y < 0) {
      y = (integral_y - 0.5) * planning_tree_resolution_;
    } else {
      y = point.y();
    }
  } else {
    if (integral_y > 0) {
      y = (integral_y - 0.5) * planning_tree_resolution_;
    } else if (integral_y < 0) {
      y = (integral_y + 0.5) * planning_tree_resolution_;
    } else {
      y = point.y();
    }
  }

  if ( std::fabs(point.z() - integral_z * planning_tree_resolution_) >= planning_tree_resolution_ / 2) {
    if (integral_z > 0) {
      z = (integral_z + 0.5) * planning_tree_resolution_;
    } else if (integral_z < 0) {
      z = (integral_z - 0.5) * planning_tree_resolution_;
    } else {
      z = point.z();
    }
  } else {
    if (integral_z > 0) {
      z = (integral_z - 0.5) * planning_tree_resolution_;
    } else if (integral_z < 0) {
      z = (integral_z + 0.5) * planning_tree_resolution_;
    } else {
      z = point.z();
    }
  }
  octomap::point3d re_point(x, y, z);
  // ROS_INFO_STREAM("x, y, z : " << re_point.x() << ", " << re_point.y() << ", " << re_point.z() );

  return re_point;
}

bool JPSPlanner::checkAvailability(octomap::point3d &point)
{
  octomap::OcTreeKey   tentative_key  = octree_->coordToKey(point);
  octomap::OcTreeNode* tentative_node = octree_->search(tentative_key);

  if (tentative_node == NULL) {
    printf(ANSI_COLOR_RED "point is not on the grid!" ANSI_COLOR_RESET "\n");
    return false;
  }

  if (isOccupied(tentative_node)) {
    printf(ANSI_COLOR_RED "point is occupied!" ANSI_COLOR_RESET "\n");

    return false;
  } 
  else if (isOutside(point)) {
    printf(ANSI_COLOR_RED "point is outside!" ANSI_COLOR_RESET "\n");
    std::cout << ANSI_COLOR_MAGENTA "coordinate: (" << point.x() << "," << point.y() << "," << point.z() << ")" ANSI_COLOR_RESET << std::endl;
    printf(ANSI_COLOR_MAGENTA "octomap region: x=%f, y=%f, z=%f" ANSI_COLOR_RESET "\n", xDim_, yDim_, zDim_);

    return false;
  }

  return true;
}

template <typename T>
bool JPSPlanner::parse_param(const std::string &param_name, T &param_dest) 
{
  if (!nh_.getParam(param_name, param_dest)) {
    if (planner_verbose_) {
      printf(ANSI_COLOR_RED "[rokae_JPS_planner]: Could not load param '%s'." ANSI_COLOR_RESET "\n", param_name.c_str() );
    }

    return false;
  } 
  else {
    if (planner_verbose_) {
      std::cout << ANSI_COLOR_BLUE "[rokae_JPS_planner]: Loaded '" << param_name << "' = '" << param_dest << "'" ANSI_COLOR_RESET << std::endl;
    }

    return true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rokae_jps_planner");

  ros::AsyncSpinner spinner(0); 
  spinner.start();

  ros::NodeHandle nh;
  JPSPlanner rokae_jps_planner(&nh);

  ros::waitForShutdown();

  return 0;
}