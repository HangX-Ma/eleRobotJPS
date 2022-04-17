#include "elerobot_tcpsocket_client.hpp"

using namespace eleRobot;

constexpr int            ERROR       = -1;
constexpr unsigned short DES_PORT    = 5001;
std::string              SERVER_IP   = "192.168.1.159";
const char* recv_msg = NULL;
int         recv_len = -1;

socket_client::socket_client(ros::NodeHandle *nodehandle) 
  :nh_(*nodehandle)
{
  tcpSocket = std::make_shared<TCPSocket>([](int errorCode, std::string errorMessage){
        std::cout << ANSI_COLOR_RED "Socket creation error:" << errorCode << " : " << errorMessage << ANSI_COLOR_RESET << std::endl;
    });  
  
  tcpSocket->onRawMessageReceived = [&](const char* message, int length) {
      recv_msg = message; 
      recv_len = length; 
      // std::cout <<  "Message from the Server: " << message << " Length:(" << length << ")" << std::endl;
    };

  tcpSocket->Connect(SERVER_IP, DES_PORT, [&] { 
        std::cout << ANSI_COLOR_GREEN "Connected to the server successfully." ANSI_COLOR_RESET << std::endl;},
        [](int errorCode, std::string errorMessage){
        std::cout << ANSI_COLOR_RED << errorCode << " : " << errorMessage << ANSI_COLOR_RESET << std::endl;
    });// CONNECTION FAILED
}

template<typename T>
int find_element(T const str[], T element) {
  int i;
  for (i = 0; i < std::strlen(str); i++) {
    if (element == str[i]) {
      return i;
    }
  }
  return ERROR;
}

//! 2.1 get current angles of robot 
std::vector<double> socket_client::get_angles() {
  recv_msg = NULL;
  std::string command = "get_angles();";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

  // find the `[` address and move the pointer to the next location
  std::vector<double> angles;
  char *endPtr = NULL;
  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, '['); 
  char * currPtr = (char *)(currPtrTmp);

  while (*currPtr != ']') {
    currPtr++;
    double joint_config = std::strtof(currPtr, &endPtr);
    angles.push_back(joint_config);
    currPtr = endPtr;
  }

  if (angles.size() != 6) {
    perror("Joint angles number is wrong");

    return std::vector<double>(); 
  }

  if (angles.at(0) == -1.0 && angles.at(1) == -2.0 && angles.at(2) == -3.0 && 
                              angles.at(3) == -4.0 && angles.at(4) == -1.0 && angles.at(5) == -1.0) {
    perror("Error occurred. InvalidAngles() received.");

    return std::vector<double>(); 
  }

  return angles;
}

//! 2.2 set the angles of robot
void socket_client::set_angles(double j1, double j2, double j3, double j4, double j5, double j6, int speed){
  recv_msg = NULL;
  const char* format = "set_angles(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, j1,j2,j3,j4,j5,j6,speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

}

//! 2.3 set the angle of robot
void socket_client::set_angle(int joint, double angle, int speed){
  recv_msg = NULL;
  const char* format = "set_angle(J%d,%.3f,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, joint, angle, speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.4 get current coordinates of robot
std::vector<double> socket_client::get_coords() {
  recv_msg = NULL;
  std::string command = "get_coords();";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

  // find the `[` address and move the pointer to the next location
  std::vector<double> coords;
  char *endPtr = NULL;
  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, '['); 
  char * currPtr = (char *)(currPtrTmp);

  while (*currPtr != ']') {
    currPtr++;
    double joint_config = std::strtof(currPtr, &endPtr);
    coords.push_back(joint_config);
    currPtr = endPtr;
  }

  if (coords.size() != 6) {
    perror("Coords number is wrong");

    return std::vector<double>(); 
  }

  if (coords.at(0) == -1.0 && coords.at(1) == -2.0 && coords.at(2) == -3.0 && 
                              coords.at(3) == -4.0 && coords.at(4) == -1.0 && coords.at(5) == -1.0) {
    perror("Error occurred. InvalidCoords() received.");

    return std::vector<double>(); 
  }

  return coords;
}

//! 2.5 set the coordinates of robot
void socket_client::set_coords(double axis_x_coord, double axis_y_coord, double axis_z_coord, double axis_rx_coord, double axis_ry_coord, double axis_rz_coord, int speed) {
  recv_msg = NULL;
  const char* format = "set_coords(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, axis_x_coord, axis_y_coord, axis_z_coord, axis_rx_coord, axis_ry_coord, axis_rz_coord, speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.6 set the coordinate of one axis
void socket_client::set_coord(char axis, double coordinate , int speed) {
  recv_msg = NULL;
  const char* format = "set_angle(%c,%.3f,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, axis, coordinate, speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}


//! 2.7 get the signal of digital out pin 
bool socket_client::get_digital_out(int pin_num) {
  recv_msg = NULL;
  const char* format = "get_digital_out(%d)";
  char command_conv[1024];
  sprintf(command_conv, format, pin_num);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int res = atoi(currPtr);
  if (res == 1) {
    return true;
  } 
  else {
    return false; 
  } // ERROR
}

//! 2.8 set the signal of digital out pin 
bool socket_client::set_digital_out(int pin_num, int signal) {
  recv_msg = NULL;
  const char* format = "get_digital_out(%d,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, pin_num, signal);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.9 get the signal of digital in pin
bool socket_client::get_digital_in(int pin_num) {
  recv_msg = NULL;
  const char* format = "get_digital_in(%d)";
  char command_conv[1024];
  sprintf(command_conv, format, pin_num);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int res = atoi(currPtr);
  if (res == 1) {
    return true;
  } 
  else {
    return false; 
  } // ERROR
}

//! 2.10 set the signal of analog out pin 
bool socket_client::set_analog_out(int pin_num, int signal) {
  recv_msg = NULL;
  const char* format = "set_analog_out(%d,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, pin_num, signal);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.11 change the coordinate of one axis in one direction continuously 
void socket_client::jog_coord(char axis, int dir, int speed){
  recv_msg = NULL;
  const char* format = "jog_coord(%c,%d,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, axis, dir, speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.12 change the angle of one joint in one direction continuously 
void socket_client::jog_angle(int joint, int dir, int speed){
  recv_msg = NULL;
  const char* format = "jog_angle(J%d,%d,%d)";
  char command_conv[1024];
  sprintf(command_conv, format, joint, dir, speed);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.13 enable the system 
void socket_client::state_on() {
  recv_msg = NULL;
  std::string command = "state_on()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.14 disable the system 
void socket_client::state_off() {
  recv_msg = NULL;
  std::string command = "state_off()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.15 stop the task 
void socket_client::task_stop() {
  recv_msg = NULL;
  std::string command = "task_stop()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.16 set feed rate 
bool socket_client::set_feed_rate(double rate) {
  recv_msg = NULL;
  const char* format = "set_feed_rate(%.2f)";
  char command_conv[1024];
  sprintf(command_conv, format, rate);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int res = atoi(currPtr);
  if (res == 0) {
    return true;
  }
  else {
    return false;
  }
}

//! 2.16 set feed rate 
void socket_client::wait(double sec) {
  recv_msg = NULL;
  const char* format = "wait(%.2f)";
  char command_conv[1024];
  sprintf(command_conv, format, sec);
  std::string command(command_conv);

  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}


//! 2.19 power on the robot 
void socket_client::power_on() {
  recv_msg = NULL;
  std::string command = "power_on()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.20 power off the robot 
void socket_client::power_off() {
  recv_msg = NULL;
  std::string command = "power_off()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.21 get the speed the robot
int socket_client::get_speed() {
  recv_msg = NULL;
  std::string command = "get_speed()";
  tcpSocket->Send(command);
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int speed = atoi(currPtr);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
  return speed;
}

//! 2.22 check the state of the robot 
void socket_client::state_check() {
  recv_msg = NULL;
  std::string command = "state_check()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.23 check if the robot is running
std::pair<bool, int> socket_client::check_running() {
  recv_msg = NULL;
  std::string command = "check_running()";
  tcpSocket->Send(command);
  while (recv_msg == NULL) {
    ;
  }

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int res = atoi(currPtr);
  if (res == 1) {
    printf(ANSI_COLOR_YELLOW "Robot is running\n" ANSI_COLOR_RESET);

    return std::make_pair(true, res);
  } 
  else if (res == 0) {
    printf(ANSI_COLOR_GREEN "Robot is not running\n" ANSI_COLOR_RESET);

    return std::make_pair(false, res);
  } 
  else {
    printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);

    return std::make_pair(false, ERROR); 
  }
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.24 set the torque limit of the robot 
void socket_client::set_torque_limit(char axis, double torque) {
  recv_msg = NULL;
  const char* format = "set_torque_limit(%c,%.3f)";
  char command_conv[1024];
  sprintf(command_conv, format, axis, torque);
  std::string command(command_conv);

  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.26 get the robot error 
void socket_client::read_next_error() {
  recv_msg = NULL;
  std::string command = "read_next_error()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.27 set the payload of the robot 
void socket_client::set_payload(double payload) {
  recv_msg = NULL;
  const char* format = "set_payload(%.2f)";
  char command_conv[1024];
  sprintf(command_conv, format, payload);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.28 set the acceleration of the robot 
void socket_client::set_acceleration(int acc) {
  recv_msg = NULL;
  const char* format = "set_acceleration(%d)";
  char command_conv[1024];
  sprintf(command_conv, format, acc);
  std::string command(command_conv);
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.29 get the acceleration of the robot 
int socket_client::get_acceleration() {
  recv_msg = NULL;
  std::string command = "get_acceleration()";
  tcpSocket->Send(command);
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);

  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int acc = atoi(currPtr);

  // wait command done
  while(!wait_command_done()) {
    ;
  }

  return acc;
}

//! 2.32 wait for command done 
bool socket_client::wait_command_done() {
  std::lock_guard<std::mutex> lock(lock_);
  recv_msg = NULL;
  std::string command = "wait_command_done()";
  tcpSocket->Send(command);
  while (recv_msg == NULL) {
    ;
  }
  // printf(ANSI_COLOR_YELLOW "%s\n" ANSI_COLOR_RESET, recv_msg);
  const char *currPtrTmp = recv_msg + find_element<char>(recv_msg, ':') + 1; 
  char * currPtr = (char *)(currPtrTmp);
  int res = atoi(currPtr);
  if (res == 0) {
    return true;
  } 
  else {
    return false; 
  }
}

//! 2.32 pause the program 
void socket_client::pause_program() {
  recv_msg = NULL;
  std::string command = "pause_program()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}

//! 2.33 resume the program 
void socket_client::resume_program() {
  recv_msg = NULL;
  std::string command = "resume_program()";
  tcpSocket->Send(command);
  // wait service message
  while (recv_msg == NULL) {
    ;
  }
  printf(ANSI_COLOR_CYAN "%s\n" ANSI_COLOR_RESET, recv_msg);
  // wait command done
  while(!wait_command_done()) {
    ;
  }
}


// TEST CODE

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "elerobot_socket_control");
//   ros::NodeHandle nh("~");
//   socket_client sockClient(&nh);

//   // sockClient.power_on();
  
//   // sockClient.state_on();

//   // std::vector<double> angles = sockClient.get_angles();
//   // sockClient.set_angles(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]-10, 100);
//   // sockClient.set_angle(6, angles[5]-10, 100);
//   // sockClient.get_coords();
//   // sockClient.get_digital_out(1);
//   // sockClient.get_digital_in(0);
//   // sockClient.state_check();
//   // sockClient.check_running();
//   // sockClient.resume_program();

//   // sockClient.pause_program();
//   // sockClient.wait_command_done();
//   // sockClient.power_off();
//   // sockClient.power_on();
//   // sockClient.state_on();
//   // sockClient.state_off();  
//   // sockClient.task_stop();
//   // sockClient.set_feed_rate(50);
//   // sockClient.read_next_error();
//   // sockClient.wait(2);
//   // sockClient.get_speed();
//   // sockClient.set_torque_limit('x',10.0);
//   // sockClient.set_payload(5.0);
//   // sockClient.set_acceleration(400);
//   // sockClient.get_acceleration();

//   ros::spin();

//   return 0;
// }

// subscribe the JPS planner service
// JPS_PLANNING    = "/rokae_arm/goto_trigger";

// ros::service::waitForService(JPS_PLANNING);
// planner_client_ = nh_.serviceClient<rokae_jps_navigation::Goto>(JPS_PLANNING);
// ROS_INFO(ANSI_COLOR_GREEN "Planner is loaded." ANSI_COLOR_RESET);