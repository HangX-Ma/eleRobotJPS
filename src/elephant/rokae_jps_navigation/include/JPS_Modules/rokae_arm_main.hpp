/**
 * @file rokae_arm_main.hpp
 * @brief manipulator controller
 * @details This file is used to control the manipulator and output the result to local disk for actual manipulator usage. 
 * @author MContour (m-contour@qq.com)
 * @version v8.0

 * @copyright Copyright (c) 2021-2022 MContour. All rights reserved.
 * 
 * Copyright 2021-2022 MContour
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ROKAE_ARM_MAIN_HPP
#define ROKAE_ARM_MAIN_HPP

#include <ros/ros.h>
#include <map>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "JPS_Basis/rokae_jps_basis.hpp"
#include "rokae_pick_place_with_vacuum_gripper/GripperState.h"
#include "rokae_jps_navigation/Goto.h"

namespace execution
{

/**
 * @brief state of the vacuum gripper at the end effector
 * 
 */
std::map<std::string, bool> vacuum_gripper_state = {
              { "on", true },  //!< gripper \c on equal to value \c true
              { "off", false } //!< gripper \c off equal to value \c false
};

class operation
{
  public:
    /**
     * @brief Construct a new operation object
     * 
     * @param nodehandle ROS node handler
     */
    operation(ros::NodeHandle *nodehandle);
  private:
    ros::NodeHandle                                                          nh_;
    ros::Publisher                                                           command_pub_;              //!< command publisher for action service
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_action_client_; //!< trajectory action operation client
    ros::ServiceClient                                                       planner_client;            //!< planner operation client
    ros::ServiceClient                                                       gripper_client;            //!< gripper operation client
    std::string                                                              JPS_PLANNING;              //!< JPS service name
    std::string                                                              VACUUM_GRIPPER;            //!< vacuum gripper service name

    bool ifconfig_     = false; //!< if current operation process will use local configrations or not
    bool ifMoveback_   = false; //!< if the manipulator will move back or not
    bool cfg_loaded_   = false; //!< a flag indicates if the config file loaded successfully
  public:
    //! place the object
    void place();

    //! pick the object
    void pick();

    //! test JPS planner 
    void planner_test();

    //! test planner and movement actionlib 
    void plan_with_move();

    //! online movement operation
    void move_config(std::vector<double> &position_, std::vector<double> &velocity_, std::vector<double> &acceleration_, std::vector<double> &time_);
    
    //! local configrations loader for movement    
    bool load_config(std::vector<std::string> cfg_filePath, std::vector<double> &position_, 
                                                    std::vector<double> &velocity_, std::vector<double> &acceleration_, std::vector<double> &time_);
    //! set move back state
    void setBack(bool state=false);

    void pathPointsPlot(std::vector<double>& coord_x, std::vector<double>& coord_y, std::vector<double>& coord_z);
};
}

#endif