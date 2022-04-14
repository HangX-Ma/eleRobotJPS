/**
 * @file elerobot_tcpsocket_client.hpp
 * @brief socket client for elephant Panda3 control
 * @author m-contour
 * @date 2021-2022
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ELEROBOT_TCPSOCKET_CLIENT_HPP
#define ELEROBOT_TCPSOCKET_CLIENT_HPP
//! Set red font in printf function
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
//! Set yellow font in printf function
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
//! Set green font in printf function
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
//! Set cyan font in printf function
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
//! Set magenta font in printf function
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
//! Reset font color in printf function
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif
#include <iostream>
#include <ros/ros.h>
#include <cstring>

#include "elerobot_tcpsocket.hpp"
#include "rokae_jps_navigation/Goto.h"

namespace eleRobot{


class socket_client
{
  public:
    socket_client(ros::NodeHandle *nodehandle);

    //! 2.1 get current angles of robot 
    std::vector<double> get_angles();

    //! 2.2 set the angles of robotx`
    void set_angles(double j1, double j2, double j3, double j4, double j5, double j6, int speed);

    //! 2.3 set the angle of one joint
    void set_angle(int joint, double angle, int speed);

    //! 2.4 get current coordinates of robot
    std::vector<double> get_coords();

    //! 2.5 set the coordinates of robot
    void set_coords(double axis_x_coord, double axis_y_coord, double axis_z_coord, double axis_rx_coord, double axis_ry_coord, double axis_rz_coord, int speed);
    
    //! 2.6 set the coordinate of one axis
    void set_coord(char axis, double coordinate , int speed);
    
    //! 2.7 get the signal of digital out pin 
    bool get_digital_out(int pin_num);

    //! 2.8 set the signal of digital out pin 
    bool set_digital_out(int pin_num, int signal);

    //! 2.9 get the signal of digital in pin
    bool get_digital_in(int pin_num);

    //! 2.10 set the signal of analog out pin 
    bool set_analog_out(int pin_num, int signal);

    //! 2.11 change the coordinate of one axis in one direction continuously 
    void jog_coord(char axis, int dir, int speed);

    //! 2.12 change the angle of one joint in one direction continuously 
    void jog_angle(int joint, int dir, int speed);

    //! 2.13 enable the system 
    void state_on();

    //! 2.14 disable the system 
    void state_off();

    //! 2.15 stop the task 
    void task_stop();

    //! 2.16 set feed rate 
    bool set_feed_rate(double rate);

    //! 2.17 make the robot ‘sleep’ for some seconds 
    void wait(double sec);

    //! 2.18 mount the robot upside down 
    bool set_upside_down() = delete;

    //! 2.19 power on the robot 
    void power_on();

    //! 2.20 power off the robot 
    void power_off();

    //! 2.21 get the speed the robot
    int get_speed();

    //! 2.22 check the state of the robot 
    void state_check();

    //! 2.23 check if the robot is running
    std::pair<bool, int> check_running();

    //! 2.24 set the torque limit of the robot 
    void set_torque_limit(char axis, double torque);

    //! 2.26 get the robot error 
    void read_next_error();

    //! 2.27 set the payload of the robot 
    void set_payload(double payload);

    //! 2.28 set the acceleration of the robot 
    void set_acceleration(int acc);

    //! 2.29 get the acceleration of the robot 
    int get_acceleration();

    //! 2.30 assign variable 
    bool assign_variable() = delete;

    //! 2.31 get the value of a variable 
    bool get_variable() = delete;

    //! 2.32 wait for command done 
    bool wait_command_done();

    //! 2.32 pause the program 
    void pause_program();

    //! 2.33 resume the program 
    void resume_program();

  private: 
    ros::NodeHandle            nh_;             //!< ROS node handler
    std::shared_ptr<TCPSocket> tcpSocket;       //!< TCP socket class
    // ros::ServiceClient         planner_client_; //!< JPS planner client
    // std::string                JPS_PLANNING;    //!< JPS service name
};

}

#endif

