/**
 * @file elerobot_socket_control.hpp
 * @brief socket client for elephant Panda3 control
 * @author m-contour
 * @date 2021-2022
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ELEROBOT_SOCKET_CONTROL_HPP
#define ELEROBOT_SOCKET_CONTROL_HPP

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

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "rokae_jps_navigation/Goto.h"
#include "elerobot_socket_command.hxx"

namespace eleRobot{


class socket_client : private base_command
{
  public:
    socket_client(ros::NodeHandle *nodehandle);
    bool robot_init();
  private:    
    ros::NodeHandle nh_;
    ros::ServiceClient planner_client_; //!< JPS planner client
    std::string        JPS_PLANNING;    //!< JPS service name
};

}


#endif