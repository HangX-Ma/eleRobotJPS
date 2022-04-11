#include <ros/ros.h>
#include "elerobot_socket_control.hpp"

using namespace eleRobot;

constexpr int DES_PORT    = 5001;
constexpr int BUFFER_SIZE = 1024;
char*         SERVER_IP   = "192.168.1.100";

char send_buf[BUFFER_SIZE];
char recv_buf[BUFFER_SIZE];

socket_client::socket_client(ros::NodeHandle *nodehandle) 
  :nh_(*nodehandle)
{
  // establish TCP socket connection between PC and robot.  
  socket_cli_ = socket(AF_INET, SOCK_STREAM, 0);
  if(socket_cli_ < 0) {
    ROS_ERROR( "socket() error");
    ros::shutdown();
  }
	struct sockaddr_in server_addr;  
	memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port   = htons(DES_PORT); // Host to Network Short
  server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  ROS_INFO(ANSI_COLOR_MAGENTA "Robot connecting..." ANSI_COLOR_RESET);

  if (connect(socket_cli_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    ROS_ERROR("Robot connection error.");
    ros::shutdown();
  } 
  else {
    ROS_INFO(ANSI_COLOR_GREEN "Robot connects successfully!" ANSI_COLOR_RESET);
  }

  // subscribe the JPS planner service
  JPS_PLANNING    = "/rokae_arm/goto_trigger";

  ros::service::waitForService(JPS_PLANNING);
  planner_client_ = nh_.serviceClient<rokae_jps_navigation::Goto>(JPS_PLANNING);
  ROS_INFO(ANSI_COLOR_GREEN "Planner is loaded." ANSI_COLOR_RESET);
 
}

bool socket_client::robot_init(){
  
  send_len_ = send(socket_cli_,send_buf, std::strlen(send_buf),0);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "elerobot_socket_control");
  ros::NodeHandle nh("~");




}