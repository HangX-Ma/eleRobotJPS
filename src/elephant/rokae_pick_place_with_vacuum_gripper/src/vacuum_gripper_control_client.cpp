/**
 * @file vacuum_gripper_control_client.cpp
 * @author MContour (m-contour@qq.com)
 * @brief vacuum gripper control
 * @version 0.1
 * @date 2022-05-13
 * 
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
#include <ros/ros.h>
#include <map>
#include "rokae_pick_place_with_vacuum_gripper/GripperState.h"


std::map<std::string, bool> vacuum_gripper_state = {
              { "on", true },
              { "off", false }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vaccum_gripper_control_client");

  ros::NodeHandle nh;
  ros::ServiceClient gripper_client = nh.serviceClient<rokae_pick_place_with_vacuum_gripper::GripperState>("/rokae_arm/vaccum_gripper_control_server");

  rokae_pick_place_with_vacuum_gripper::GripperState srv;

  if (argc > 1 && argc < 3)
  {
    std::string input_arg = (std::string)argv[1];
    auto switch_state = vacuum_gripper_state.find(input_arg);
    if (switch_state != vacuum_gripper_state.end())
    {
      ROS_INFO("Switch vacuum gripper state: %s", input_arg.c_str());
      srv.request.switch_state = vacuum_gripper_state[input_arg];
      ROS_INFO("current request msg: %s", srv.request.switch_state ? "on" : "off");
    }
    else
    {
      ROS_ERROR("Invalid parament parsed to vacuum gripper control server. use 'on' or 'off'.");
      return 1;
    }
  }
  else
  {
    ROS_ERROR("The number of the parament is limited to 1.");
    return 1;
  }

  ros::service::waitForService("/rokae_arm/vaccum_gripper_control_server");
  bool success = gripper_client.call(srv);
  if (success)
  {
    ROS_INFO("Feedback from vacuum_gripper_control server :%s", srv.response.feedback.c_str());
  }
  else 
  {
    ROS_ERROR("Failed to call vacuum_gripper_control service.");
    return 1;
  }
  
  ros::spinOnce();

  return 0;
}


