/**
 * @file elerobot_socket_command.hxx
 * @brief elephant robot socket command
 * @author m-contour
 * @date 2021-2022
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ELEROBOT_SOCKET_COMMAND_HXX
#define ELEROBOT_SOCKET_COMMAND_HXX

#include <string>
#include <vector>

namespace eleRobot{

enum class command
{
  GET_ANGLES = 0,
  SET_ANGLES,
  SET_ANGLE,
  GET_COORDS,
  SET_COORDS,
  SET_COORD,
  GET_DIGIT_OUT_PIN,
  SET_DIGIT_OUT_PIN,
  GET_DIGIT_IN_PIN,
  SET_DIGIT_IN_PIN,
  SET_ANALOG_OUT_PIN,
  SET_COORD_CONTINUOUSLY,
  SET_ANGLE_CONTINUOUSLY,
  ENABLE_SYS,
  DISABLE_SYS,
  STOP_TASK,
  SET_FEED_RATE,
  SLEEP,
  SET_UPSIDE_DOWN,
  POWER_ON,
  POWER_OFF,
  CHECK_ROBOT_STATE,
  CHECK_RUNNING,
  SET_TORQUE_LIMIT,
  GET_ERROR_INFO,
  SET_PAYLOAD,
  SET_ACC,
  GET_ACC,
  ASSIGN_VAR,
  GET_VAR,
  WAIT_COMMAND_DONE,
  PAUSE,
  RESUME,
};

struct timeval timeout;
constexpr int ERROR = -1;

/**
 * @brief command of elephant Panda3 using socket communication protocol
 * 
 * @ref [RoboFlowScript_Manual/3-RoboFlow CommunicationSocket.pdf]: Tutorial of Communicate with Elephant Robot through Socket
 */
class base_command{
  public:
    base_command();

    //! 2.1 get current angles of robot 
    std::vector<double> get_angles(char *buf);

    //! 2.2 set the angles of robot
    bool set_angles(char *buf, double j1, double j2, double j3, double j4, double j5, double j6, double speed);

    //! 2.3 set the angle of one joint
    bool set_angle(char *buf);

    //! 2.4 get current coordinates of robot
    bool get_coords(char *buf);

    //! 2.5 set the coordinates of robot
    bool set_coords(char *buf);
    
    //! 2.6 set the coordinate of one axis
    bool set_coord(char *buf);
    
    //! 2.7 get the signal of digital out pin 
    bool get_digital_out(char *buf);

    //! 2.8 set the signal of digital out pin 
    bool set_digital_out(char *buf);

    //! 2.9 get the signal of digital in pin
    bool get_digital_in(char *buf);

    //! 2.10 set the signal of analog out pin 
    bool set_analog_out(char *buf);

    //! 2.11 change the coordinate of one axis in one direction continuously 
    bool jog_coord(char *buf);

    //! 2.12 change the angle of one joint in one direction continuously 
    bool jog_angle(char *buf);

    //! 2.13 enable the system 
    bool state_on(char *buf);

    //! 2.14 disable the system 
    bool state_off(char *buf);

    //! 2.15 stop the task 
    bool task_stop(char *buf);

    //! 2.16 set feed rate 
    bool set_feed_rate(char *buf);

    //! 2.17 make the robot ‘sleep’ for some seconds 
    bool wait(char *buf);

    //! 2.18 mount the robot upside down 
    bool set_upside_down(char *buf);

    //! 2.19 power on the robot 
    bool power_on(char *buf);

    //! 2.20 power off the robot 
    bool power_off(char *buf);

    //! 2.21 get the speed the robot
    bool get_speed(char *buf);

    //! 2.22 check the state of the robot 
    bool state_check(char *buf);

    //! 2.23 check if the robot is running
    std::pair<bool, int> check_running(char *buf);

    //! 2.24 set the torque limit of the robot 
    bool set_torque_limit(char *buf);

    //! 2.26 get the robot error 
    bool read_next_error(char *buf);

    //! 2.27 set the payload of the robot 
    bool set_payload(char *buf);

    //! 2.28 set the acceleration of the robot 
    bool set_acceleration(char *buf);

    //! 2.29 get the acceleration of the robot 
    bool get_acceleration(char *buf);

    //! 2.30 assign variable 
    bool assign_variable(char *buf);

    //! 2.31 get the value of a variable 
    bool get_variable(char *buf);

    //! 2.32 wait for command done 
    bool wait_command_done(char *buf);

    //! 2.32 pause the program 
    bool pause_program(char *buf);

    //! 2.33 resume the program 
    bool resume_program(char *buf);

  public:
    int len_;
    int client_sockfd_;

  private:
    std::string socket_string_;
};

}
#endif