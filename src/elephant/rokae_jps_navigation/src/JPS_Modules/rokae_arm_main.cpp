#include "JPS_Modules/rokae_arm_main.hpp"

using namespace execution;

operation::operation(ros::NodeHandle *nodehandle):nh_(*nodehandle), trajectory_action_client_("/manipulator_controller/follow_joint_trajectory", true)
{
  // publisher
  command_pub_   = nh_.advertise<trajectory_msgs::JointTrajectory>("/manipulator_controller/command", 1);
  // service client
  JPS_PLANNING   = "/rokae_arm/goto_trigger";
  VACUUM_GRIPPER = "/rokae_arm/vaccum_gripper_control_server";

  ros::service::waitForService(JPS_PLANNING);
  ros::service::waitForService(VACUUM_GRIPPER);

  planner_client = nh_.serviceClient<rokae_jps_navigation::Goto>(JPS_PLANNING);
  gripper_client = nh_.serviceClient<rokae_pick_place_with_vacuum_gripper::GripperState>(VACUUM_GRIPPER);

}


void operation::setBack(bool state) {ifMoveback_ = state;}
// REF http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
// REF https://answers.ros.org/question/362380/publish-multiple-points-on-trajectory_msgsjointtrajectory-topic/

void operation::move_config(std::vector<double> &position_, std::vector<double> &velocity_, std::vector<double> &acceleration_, std::vector<double> &time_)
{
  printf(ANSI_COLOR_CYAN "[rokae_jps_operation]: move manipulator" ANSI_COLOR_RESET "\n");

  printf(ANSI_COLOR_BLUE "Waiting for action server to start." ANSI_COLOR_RESET "\n");
  trajectory_action_client_.waitForServer();
  printf(ANSI_COLOR_BLUE "Action server started, sending goal." ANSI_COLOR_RESET "\n");

  if (trajectory_action_client_.isServerConnected()) {
    printf(ANSI_COLOR_GREEN "[move]: client connect to the action server" ANSI_COLOR_RESET "\n");
  } else {
    printf(ANSI_COLOR_RED "[move]: client cannot connect to the action server" ANSI_COLOR_RESET "\n");
  }

  control_msgs::FollowJointTrajectoryAction action;
  auto time_stamp                                    = ros::Time().now();
  action.action_goal.goal.trajectory.joint_names     =  {"rokae_arm_joint0", "rokae_arm_joint1", "rokae_arm_joint2", 
                                                                            "rokae_arm_joint3", "rokae_arm_joint4", "rokae_arm_joint5"};
  action.action_goal.goal.trajectory.header.frame_id = "world";
  action.action_goal.goal.trajectory.header.stamp    = time_stamp;

  int    data_size                                   = time_.size();
  int    nDof                                        = 6;
  double time_interval                               = time_.at(1) - time_.at(0);
  double time_dur                                    = 0;

  for (int i = 0; i < data_size; i++)
  {
    time_dur  += time_interval;
    trajectory_msgs::JointTrajectoryPoint joint_traj_point;
    for (int j = 0; j < nDof; j++)
    {
      // make rokae_arm_joint5 fixed
      if (j == nDof - 1) {
        joint_traj_point.positions.push_back(0);
        joint_traj_point.velocities.push_back(0);
        joint_traj_point.accelerations.push_back(0);
        joint_traj_point.time_from_start = ros::Duration(time_dur);          
      } 
      else {
        joint_traj_point.positions.push_back(position_.at(i*nDof+j));
        joint_traj_point.velocities.push_back(velocity_.at(i*nDof+j));
        joint_traj_point.accelerations.push_back(acceleration_.at(i*nDof+j));
        joint_traj_point.time_from_start = ros::Duration(time_dur);
      }
    }
    action.action_goal.goal.trajectory.points.push_back(joint_traj_point);
  }

  trajectory_action_client_.sendGoal(action.action_goal.goal);
  bool finished_before_timeout = trajectory_action_client_.waitForResult(ros::Duration(60.0));
  
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = trajectory_action_client_.getState();
    printf(ANSI_COLOR_GREEN "[move]: Action finished: %s" ANSI_COLOR_RESET "\n",state.toString().c_str());
  }
  else {
    printf(ANSI_COLOR_RED "[move]: Action did not finish before the time out." ANSI_COLOR_RESET "\n");
  }

}


void operation::pick()
{
  if (ifconfig_) {
    printf(ANSI_COLOR_RED "[operation error]: Cannot use the <pick> function, configurations had been loaded." ANSI_COLOR_RESET "\n");

    return;
  }

  printf(ANSI_COLOR_CYAN "[rokae_jps_operation]: begin <pick> process" ANSI_COLOR_RESET "\n");

  rokae_jps_navigation::Goto                         planner_srv;
  rokae_pick_place_with_vacuum_gripper::GripperState gripper_srv;

  geometry_msgs::Pose pose;
  // clear previous configuration
  planner_srv.request.goal_pose.clear();
  planner_srv.request.ifback = ifMoveback_;

  // middle_pose2
  pose.position.x = 0.175002;
  pose.position.y = 0.0;
  pose.position.z = 1.27292;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.7071;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.7071;
  planner_srv.request.goal_pose.push_back(pose);

  // TODO middle to grasping
  pose.position.x = -0.232124;
  pose.position.y = 0.0;
  pose.position.z = 1.00138;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.7071;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.7071;
  planner_srv.request.goal_pose.push_back(pose);

  // pre_grasp_pose2
  pose.position.x = -0.232124;
  pose.position.y = 0.0;
  pose.position.z = 1.00138;
  pose.orientation.w = 0.0;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  planner_srv.request.goal_pose.push_back(pose);

  // grasp_pose2
  pose.position.x = -0.235698;
  pose.position.y = 0.0;
  pose.position.z = 0.868984;
  pose.orientation.w = 0.0;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  planner_srv.request.goal_pose.push_back(pose);

  if (planner_client.call(planner_srv)) {
    if(planner_srv.response.success) {
      ROS_INFO_STREAM(planner_srv.response.message);
      printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING OPERATION" ANSI_COLOR_RESET "\n");
      move_config(planner_srv.response.pos, planner_srv.response.vel, planner_srv.response.acc, planner_srv.response.t);

      // gripper on
      gripper_srv.request.switch_state = true;
      if (gripper_client.call(gripper_srv)) {
        printf(ANSI_COLOR_MAGENTA "Feedback from vacuum_gripper_control server : %s" ANSI_COLOR_RESET "\n", gripper_srv.response.feedback.c_str());
        ros::Duration(5).sleep(); // to stabilize the PCB
      }
      else {
        printf(ANSI_COLOR_RED "Failed to call vacuum_gripper_control service." ANSI_COLOR_RESET "\n");
      }
    } 
    else {
      printf(ANSI_COLOR_RED "JPS planner error" ANSI_COLOR_RESET "\n");
    }

    // moving back?
    if (ifMoveback_) {
      // TODO back move
      printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING BACK OPERATION" ANSI_COLOR_RESET "\n");
      move_config(planner_srv.response.back_pos, planner_srv.response.back_vel, planner_srv.response.back_acc, planner_srv.response.back_t);
      ros::Duration(5).sleep(); // to stabilize the PCB
    }
  } 
  else {
    printf(ANSI_COLOR_RED "Failed to call JPS planner" ANSI_COLOR_RESET "\n");
  }
}

void operation::place()
{
  if (ifconfig_) {
    printf(ANSI_COLOR_RED "[rokae_jps_operation]: Cannot use the <place> function, configurations had been loaded." ANSI_COLOR_RESET "\n");
    return;
  }

  printf(ANSI_COLOR_CYAN "[rokae_jps_operation]: begin <place> process" ANSI_COLOR_RESET "\n");

  rokae_jps_navigation::Goto planner_srv;
  rokae_pick_place_with_vacuum_gripper::GripperState gripper_srv;

  geometry_msgs::Pose pose;

  // clear previous configuration
  planner_srv.request.goal_pose.clear();
  planner_srv.request.ifback = ifMoveback_;

  // pre_pose2
  pose.position.x    = 0.731675;
  pose.position.y    = 0.0;
  pose.position.z    = 0.971591;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.707107;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.707107;
  planner_srv.request.goal_pose.push_back(pose);

  // put_pose2
  pose.position.x    = 0.643245;
  pose.position.y    = 0.0;
  pose.position.z    = 0.883091;
  pose.orientation.w = 0.0;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = -0.000398167;
  planner_srv.request.goal_pose.push_back(pose);

  if (planner_client.call(planner_srv)) {
    if(planner_srv.response.success) {
      ROS_INFO_STREAM(planner_srv.response.message);
      printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING OPERATION" ANSI_COLOR_RESET "\n");
      move_config(planner_srv.response.pos, planner_srv.response.vel, planner_srv.response.acc, planner_srv.response.t);

      // gripper off
      gripper_srv.request.switch_state = false;
      if (gripper_client.call(gripper_srv)) {
        printf(ANSI_COLOR_MAGENTA "Feedback from vacuum_gripper_control server : %s" ANSI_COLOR_RESET "\n", gripper_srv.response.feedback.c_str());
        ros::Duration(2).sleep(); // to stabilize the PCB
      }
      else {
        printf(ANSI_COLOR_RED "Failed to call vacuum_gripper_control service." ANSI_COLOR_RESET "\n");
      }

      // moving back?
      if (ifMoveback_) {
        // TODO back move
        printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING BACK OPERATION" ANSI_COLOR_RESET "\n");
        move_config(planner_srv.response.back_pos, planner_srv.response.back_vel, planner_srv.response.back_acc, planner_srv.response.back_t);
        ros::Duration(5).sleep(); // to stabilize the PCB
      }

    } 
    else {
      printf(ANSI_COLOR_RED "JPS planner error" ANSI_COLOR_RESET "\n");
    }
  } 
  else {
    printf(ANSI_COLOR_RED "Failed to call JPS planner" ANSI_COLOR_RESET "\n");
  }

}


void operation::planner_test()
{
  if (ifconfig_) {
    printf(ANSI_COLOR_RED "[operation error]: Cannot use the <planner_test> function, configurations had been loaded." ANSI_COLOR_RESET "\n");
    return;
  }

  printf(ANSI_COLOR_CYAN "[rokae_jps_operation]: Begin <planner_test> process" ANSI_COLOR_RESET "\n");
  rokae_jps_navigation::Goto planner_srv;
  geometry_msgs::Pose pose;

  // forward
  pose.position.x = -0.62792;
  pose.position.y = 0.115;
  pose.position.z = 0.255789;
  pose.orientation.w = -0.5;
  pose.orientation.x = 0.5;
  pose.orientation.y = 0.5;
  pose.orientation.z = -0.5;
  planner_srv.request.goal_pose.push_back(pose);

  if (planner_client.call(planner_srv)) {
    if(planner_srv.response.success) {
      ROS_INFO_STREAM(planner_srv.response.message);
    } else {
      printf(ANSI_COLOR_RED "JPS planner error" ANSI_COLOR_RESET "\n");
    }
  } 
  else {
    printf(ANSI_COLOR_RED "Failed to call JPS planner" ANSI_COLOR_RESET "\n");
  }
}

void operation::plan_with_move()
{
  if (ifconfig_) {
    printf(ANSI_COLOR_RED "[operation error]: Cannot use the <planner_with_move> function, configurations had been loaded." ANSI_COLOR_RESET "\n");
    return;
  }

  printf(ANSI_COLOR_CYAN "[rokae_jps_operation]: begin <planner_with_move> process" ANSI_COLOR_RESET "\n");
  rokae_jps_navigation::Goto planner_srv;
  geometry_msgs::Pose pose;
  
  // clear previous configuration
  planner_srv.request.goal_pose.clear();
  planner_srv.request.ifback = ifMoveback_;

  // forward
  pose.position.x = -0.62792;
  pose.position.y = 0.115;
  pose.position.z = 0.255789;
  pose.orientation.w = -0.5;
  pose.orientation.x = 0.5;
  pose.orientation.y = 0.5;
  pose.orientation.z = -0.5;
  planner_srv.request.goal_pose.push_back(pose);


  if (planner_client.call(planner_srv)) {
    if(planner_srv.response.success) {
      ROS_INFO_STREAM(planner_srv.response.message);
      printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING OPERATION" ANSI_COLOR_RESET "\n");
      move_config(planner_srv.response.pos, planner_srv.response.vel, planner_srv.response.acc, planner_srv.response.t);
      if (ifMoveback_) {
        // TODO back move
        printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING BACK OPERATION" ANSI_COLOR_RESET "\n");
        move_config(planner_srv.response.back_pos, planner_srv.response.back_vel, planner_srv.response.back_acc, planner_srv.response.back_t);
        ros::Duration(5).sleep(); // to stabilize the PCB
      }
    } 
    else {
      printf(ANSI_COLOR_RED "JPS planner error" ANSI_COLOR_RESET "\n");
    }
  } 
  else {
    printf(ANSI_COLOR_RED "Failed to call JPS planner" ANSI_COLOR_RESET "\n");
  }
}

bool operation::load_config(std::vector<std::string> cfgfilepath, std::vector<double> &position_,
                               std::vector<double> &velocity_, std::vector<double> &acceleration_, std::vector<double> &time_)
{
  ifconfig_   = true;
  cfg_loaded_ = false;

  printf(ANSI_COLOR_YELLOW "config files needs to be load in order.\n 1.[position.cfg], 2.[velocity.cfg], 3.[acceleration.cfg], 4.[time.cfg]" ANSI_COLOR_RESET "\n");
  
  if (cfgfilepath.size() != 4) {
    printf(ANSI_COLOR_RED "Loading configuration file failed. (insufficient config files)" ANSI_COLOR_RESET "\n");
    return false;
  }

  std::fstream pos_cfgFile(cfgfilepath.at(0), std::ios::in);
  std::fstream vel_cfgFile(cfgfilepath.at(1), std::ios::in);
  std::fstream acc_cfgFile(cfgfilepath.at(2), std::ios::in);
  std::fstream t_cfgFile(cfgfilepath.at(3), std::ios::in);

  if(!pos_cfgFile.is_open() || !vel_cfgFile.is_open() || !acc_cfgFile.is_open() || !t_cfgFile.is_open()) {
    printf(ANSI_COLOR_RED "Some file can not be opened!" ANSI_COLOR_RESET "\n");
    return false;
  }

  std::string buffer;
  while(std::getline(pos_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      position_.push_back(data);
    }
  }

  while(std::getline(vel_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      velocity_.push_back(data);
    }
  }

  while(std::getline(acc_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      acceleration_.push_back(data);
    }
  }

  while(std::getline(t_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      time_.push_back(data);
    }
  }

  pos_cfgFile.close(); pos_cfgFile.clear();
  vel_cfgFile.close(); vel_cfgFile.clear();
  acc_cfgFile.close(); acc_cfgFile.clear();
  t_cfgFile.close();   t_cfgFile.clear();

  int  nDof                  = 6;

  std::cout << ANSI_COLOR_YELLOW "[position.cfg]: data group number (" << position_.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[velocity.cfg]: data group number (" << velocity_.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[acceleration.cfg]: data group number (" << acceleration_.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[time.cfg]: data group number (" << time_.size() << ")" ANSI_COLOR_RESET << std::endl;

  bool data_size_correctness = true;
  data_size_correctness     &= velocity_.size()/nDof == time_.size();
  data_size_correctness     &= position_.size()/nDof == time_.size();
  data_size_correctness     &= acceleration_.size()/nDof == time_.size();

  if (!data_size_correctness) {
    printf(ANSI_COLOR_RED "data group size is not correspond" ANSI_COLOR_RESET "\n");
    return false;
  }

  cfg_loaded_ = true;

  return true;
}

void clear_configuration(std::vector<double> &position_, std::vector<double> &velocity_, std::vector<double> &acceleration_, std::vector<double> &time_)
{
  position_.clear();
  velocity_.clear();
  acceleration_.clear();
  time_.clear();
  printf(ANSI_COLOR_GREEN "clear all configuration" ANSI_COLOR_RESET "\n");
  
  return;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "rokae_arm_navigation");

  std::string path_prefix_ = "/home/contour/ws_catkin_rokae/src/rokae_demo/rokae_arm_toppra/share/";
  std::string folder_name1 = "1648089318";

  std::vector<double> position_m, velocity_m, acceleration_m, time_m;
  std::vector<std::string> cfgconfig_stage1 {path_prefix_ + folder_name1 + "/toppra_joints_pos_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_vel_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_acc_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_t_" + folder_name1 + ".txt"};
  ros::NodeHandle nh;
  execution::operation controller(&nh);

  /* ------------------------- planner test module ------------------------- */
  // controller.planner_test();

  /* ------------------------- planner and movement actionlib test module ------------------------- */
  controller.plan_with_move();

  /* ------------------------- pick module ------------------------- */
  // controller.setBack(true);
  // controller.pick();

  /* ------------------------- place module ------------------------- */
  // controller.place();

  /* ------------------------- load local config files ------------------------- */
  // if (controller.load_config(cfgconfig_stage1, position_m, velocity_m, acceleration_m, time_m)) {
  //   controller.move_config(position_m, velocity_m, acceleration_m, time_m);
  // }
  // clear_configuration(position_m, velocity_m, acceleration_m, time_m);

  ros::spinOnce();

  return 0;

}