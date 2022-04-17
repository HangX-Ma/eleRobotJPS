#include "elerobot_tcpsocket_client.hpp"
#include "rokae_jps_navigation/Goto.h"
#include <fstream>
#include <chrono>

bool load_config(std::vector<std::string> cfgfilepath, std::vector<double> &position,
                               std::vector<double> &velocity, std::vector<double> &acceleration, std::vector<double> &time)
{
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
      position.push_back(data);
    }
  }

  while(std::getline(vel_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      velocity.push_back(data);
    }
  }

  while(std::getline(acc_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      acceleration.push_back(data);
    }
  }

  while(std::getline(t_cfgFile, buffer))
  {
    double data;
    std::stringstream ss(buffer);
    while (ss >> data) {
      time.push_back(data);
    }
  }

  pos_cfgFile.close(); pos_cfgFile.clear();
  vel_cfgFile.close(); vel_cfgFile.clear();
  acc_cfgFile.close(); acc_cfgFile.clear();
  t_cfgFile.close();   t_cfgFile.clear();

  int  nDof                  = 6;

  std::cout << ANSI_COLOR_YELLOW "[position.cfg]: data group number (" << position.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[velocity.cfg]: data group number (" << velocity.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[acceleration.cfg]: data group number (" << acceleration.size()/nDof << ")" ANSI_COLOR_RESET << std::endl;
  std::cout << ANSI_COLOR_YELLOW "[time.cfg]: data group number (" << time.size() << ")" ANSI_COLOR_RESET << std::endl;

  bool data_size_correctness = true;
  data_size_correctness     &= velocity.size()/nDof == time.size();
  data_size_correctness     &= position.size()/nDof == time.size();
  data_size_correctness     &= acceleration.size()/nDof == time.size();

  if (!data_size_correctness) {
    printf(ANSI_COLOR_RED "data group size is not correspond" ANSI_COLOR_RESET "\n");
    return false;
  }

  return true;
}

void clear_configuration(std::vector<double> &position, std::vector<double> &velocity, std::vector<double> &acceleration, std::vector<double> &time)
{
  position.clear();
  velocity.clear();
  acceleration.clear();
  time.clear();
  printf(ANSI_COLOR_GREEN "clear all configuration" ANSI_COLOR_RESET "\n");
  
  return;
}

void moveRobot(std::vector<double> &position, std::vector<double> &velocity, std::vector<double> &acceleration, std::vector<double> &time, eleRobot::socket_client &socket_cli) {
  size_t data_size = time.size();
  if (data_size < 2) {
    ROS_INFO(ANSI_COLOR_RED "Input data ERROR." ANSI_COLOR_RESET);
    return;
  }
  double interval  = time.at(1) - time.at(0);
  double angle_pos1, angle_pos2, angle_pos3, angle_pos4, angle_pos5, angle_pos6;
  // double angle_vel1, angle_vel2, angle_vel3, angle_vel4, angle_vel5, angle_vel6;
  socket_cli.wait(1); 
  
  // for (size_t i = 1; i < data_size; i += 6) {
  //   auto command_send_stamp = std::chrono::high_resolution_clock::now();
  //   angle_pos1 = position.at(i*6)/M_PI*180;
  //   angle_pos2 = position.at(i*6+1)/M_PI*180 - 90.0;
  //   angle_pos3 = position.at(i*6+2)/M_PI*180;
  //   angle_pos4 = position.at(i*6+3)/M_PI*180 - 90.0;
  //   angle_pos5 = position.at(i*6+4)/M_PI*180 + 90.0;
  //   angle_pos6 = position.at(i*6+5)/M_PI*180;
  //   angle_vel1 = position.at(i*6)/M_PI*180*60;
  //   angle_vel2 = position.at(i*6+1)/M_PI*180*60;
  //   angle_vel3 = position.at(i*6+2)/M_PI*180*60;
  //   angle_vel4 = position.at(i*6+3)/M_PI*180*60;
  //   angle_vel5 = position.at(i*6+4)/M_PI*180*60;
  //   angle_vel6 = position.at(i*6+5)/M_PI*180*60;
  //   while ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - command_send_stamp).count()) < interval) {
  //     socket_cli.set_angle(1, angle_pos1, angle_vel1);
  //     socket_cli.set_angle(2, angle_pos2, angle_vel2);
  //     socket_cli.set_angle(3, angle_pos3, angle_vel3);
  //     socket_cli.set_angle(4, angle_pos4, angle_vel4);
  //     socket_cli.set_angle(5, angle_pos5, angle_vel5);
  //     socket_cli.set_angle(6, angle_pos6, angle_vel6);
  //   }
  // }

  double vel;
  for (size_t i = 0; i < data_size; i++) {
    angle_pos1 = position.at(i*6)/M_PI*180;
    angle_pos2 = position.at(i*6+1)/M_PI*180 - 90.0;
    angle_pos3 = position.at(i*6+2)/M_PI*180;
    angle_pos4 = position.at(i*6+3)/M_PI*180 - 90.0;
    angle_pos5 = position.at(i*6+4)/M_PI*180 + 90.0;
    angle_pos6 = position.at(i*6+5)/M_PI*180;
    vel        = 300;
    socket_cli.set_angles(angle_pos1, angle_pos2, angle_pos3, angle_pos4, angle_pos5, angle_pos6, vel);
  }
  socket_cli.wait(1); 

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "elerobot_socket_control");
  ros::NodeHandle nh("~");

  std::string path_prefix_ = "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/share/";
  std::string folder_name1 = "1650109358";

  std::vector<double> position_m, velocity_m, acceleration_m, time_m;
  std::vector<std::string> cfgconfig_stage1 {path_prefix_ + folder_name1 + "/toppra_joints_pos_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_vel_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_acc_" + folder_name1 + ".txt",
                                              path_prefix_ + folder_name1 + "/toppra_joints_t_" + folder_name1 + ".txt"};

  // socket communication
  eleRobot::socket_client sockClient(&nh);

  // initialize the robot
  sockClient.power_on();
  sockClient.state_on();
  sockClient.state_check();
  std::pair<bool, int> res = sockClient.check_running();
  if (res.second == -1) {
    ROS_ERROR("Robot ERROR. Please check the robot.");
    ros::shutdown();
    exit(-1);
  } 
  else {
    if (res.first == true) {
      sockClient.task_stop();
      sockClient.wait(2);
    }
  }
  sockClient.set_feed_rate(100);
  sockClient.set_acceleration(400);
  sockClient.set_payload(0.5);
  printf(ANSI_COLOR_GREEN "Initialzing ..." ANSI_COLOR_RESET "\n");
  sockClient.set_angles(0.0, -90.0, 0.0, -90.0, 90.0, 0.0, 200);
  // while(!sockClient.wait_command_done()) {
  //   ;
  // }

  // IF YOU WANT TO USE THE LOCAL CONFIGURATION FILE
  /* ------------------------- load local config files ------------------------- */
  printf(ANSI_COLOR_GREEN "Start Moving ..." ANSI_COLOR_RESET "\n");

  if (load_config(cfgconfig_stage1, position_m, velocity_m, acceleration_m, time_m)) {
    moveRobot(position_m, velocity_m, acceleration_m, time_m, sockClient);
  }
  clear_configuration(position_m, velocity_m, acceleration_m, time_m);


  
  // IF YOU WANT TO USE THE ONLINE PLANNING

  // subscribe the JPS planner service
  // std::string JPS_PLANNING    = "/rokae_arm/goto_trigger";

  // ros::service::waitForService(JPS_PLANNING);
  // ros::ServiceClient planner_client = nh.serviceClient<rokae_jps_navigation::Goto>(JPS_PLANNING);
  // ROS_INFO(ANSI_COLOR_GREEN "Planner is loaded." ANSI_COLOR_RESET);

  // /* ------------------------------ online planning ----------------------------- */
  // rokae_jps_navigation::Goto planner_srv;
  // bool BACK_MOVE = false;
  // planner_srv.request.goal_pose.clear();
  // planner_srv.request.ifback = BACK_MOVE;

  // geometry_msgs::Pose pose;
  // // forward
  // pose.position.x = -0.62792;
  // pose.position.y = 0.115;
  // pose.position.z = 0.255789;
  // pose.orientation.w = -0.5;
  // pose.orientation.x = 0.5;
  // pose.orientation.y = 0.5;
  // pose.orientation.z = -0.5;
  // planner_srv.request.goal_pose.push_back(pose);

  // if (planner_client.call(planner_srv)) {
  //   if(planner_srv.response.success) {
  //     ROS_INFO_STREAM(planner_srv.response.message);
  //     printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING OPERATION" ANSI_COLOR_RESET "\n");
  //     moveRobot(planner_srv.response.pos, planner_srv.response.vel, planner_srv.response.acc, planner_srv.response.t, sockClient);
  //     ros::Duration(5).sleep();
  //     if (BACK_MOVE) {
  //       printf(ANSI_COLOR_GREEN "[move]: EXERT MOVING BACK OPERATION" ANSI_COLOR_RESET "\n");
  //       moveRobot(planner_srv.response.back_pos, planner_srv.response.back_vel, planner_srv.response.back_acc, planner_srv.response.back_t, sockClient);
  //       ros::Duration(5).sleep(); 
  //     }
  //   } 
  //   else {
  //     printf(ANSI_COLOR_RED "JPS planner error" ANSI_COLOR_RESET "\n");
  //   }
  // } 
  // else {
  //   printf(ANSI_COLOR_RED "Failed to call JPS planner" ANSI_COLOR_RESET "\n");
  // }

  ros::spin();

  return 0;
}

  // partial command examples

  // std::vector<double> angles = sockClient.get_angles();
  // sockClient.set_angles(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]-10, 100);
  // sockClient.set_angle(6, angles[5]-10, 100);
  // sockClient.get_coords();
  // sockClient.get_digital_out(1);
  // sockClient.get_digital_in(0);
  // sockClient.state_check();
  // sockClient.check_running();
  // sockClient.resume_program();
  // sockClient.pause_program();
  // sockClient.wait_command_done();
  // sockClient.power_off();
  // sockClient.power_on();
  // sockClient.state_on();
  // sockClient.state_off();  
  // sockClient.task_stop();
  // sockClient.set_feed_rate(50);
  // sockClient.read_next_error();
  // sockClient.wait(2);
  // sockClient.get_speed();
  // sockClient.set_torque_limit('x',10.0);
  // sockClient.set_payload(5.0);
  // sockClient.set_acceleration(400);
  // sockClient.get_acceleration();