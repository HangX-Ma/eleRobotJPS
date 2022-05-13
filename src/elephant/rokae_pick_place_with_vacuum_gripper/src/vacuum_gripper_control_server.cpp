/**
 * @file vacuum_gripper_control_server.cpp
 * @author MContour (m-contour@qq.com)
 * @brief vacuum gripper server 
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
#include <std_srvs/Empty.h>
#include "rokae_pick_place_with_vacuum_gripper/GripperState.h"


namespace gripper
{
typedef int gripper_index; 
gripper_index gindex[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

class gripper
{
  public:
    gripper(ros::NodeHandle* nodehandle);
    bool GraspFeedback(rokae_pick_place_with_vacuum_gripper::GripperState::Request &req, 
                                rokae_pick_place_with_vacuum_gripper::GripperState::Response &res);
    std::string trigger(bool gripper_state);
    void gripper_switch(std::string &state_, gripper_index &index_);

  private:
    ros::NodeHandle    nh_;
    ros::ServiceServer gripper_service_;
};

gripper::gripper(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
  gripper_service_ = nh_.advertiseService("/rokae_arm/vaccum_gripper_control_server", &gripper::GraspFeedback, this);
  ROS_INFO("vaccum gripper controller service ready");
}

bool gripper::GraspFeedback(rokae_pick_place_with_vacuum_gripper::GripperState::Request &req, 
                                  rokae_pick_place_with_vacuum_gripper::GripperState::Response &res)
{
  bool request_state = req.switch_state;
  std::string info = trigger(request_state);
  res.feedback = "The current gripper state: " + info;

  return true;
}

std::string gripper::trigger(bool gripper_state)
{
  std::string state_on = "on";
  std::string state_off = "off";

  if (gripper_state) {   // gripper on
    gripper_switch(state_on, gindex[1]);
    gripper_switch(state_on, gindex[2]);
    gripper_switch(state_on, gindex[3]);
    gripper_switch(state_on, gindex[4]);
    gripper_switch(state_on, gindex[5]);
    gripper_switch(state_on, gindex[6]);
    gripper_switch(state_on, gindex[7]);
    gripper_switch(state_on, gindex[8]);
    gripper_switch(state_on, gindex[9]);
    gripper_switch(state_on, gindex[10]);
    gripper_switch(state_on, gindex[11]);
    gripper_switch(state_on, gindex[12]);
    gripper_switch(state_on, gindex[13]);

    return state_on;
  } else {              // gripper off
    gripper_switch(state_off, gindex[1]);
    gripper_switch(state_off, gindex[2]);
    gripper_switch(state_off, gindex[3]);
    gripper_switch(state_off, gindex[4]);
    gripper_switch(state_off, gindex[5]);
    gripper_switch(state_off, gindex[6]);
    gripper_switch(state_off, gindex[7]);
    gripper_switch(state_off, gindex[8]);
    gripper_switch(state_off, gindex[9]);
    gripper_switch(state_off, gindex[10]);
    gripper_switch(state_off, gindex[11]);
    gripper_switch(state_off, gindex[12]);
    gripper_switch(state_off, gindex[13]);

    return state_off;
  }
}

void gripper::gripper_switch(std::string &state_, gripper_index &index_)
{
  // service name: /rokae_arm/vacuum_gripper_x/on\off
  std::string prefix_ = "/rokae_arm/vacuum_gripper_" + std::to_string(index_) + "/";
  std::string SERVICE_NAME = prefix_ + state_;

  ros::ServiceClient gripperClient = nh_.serviceClient<std_srvs::Empty>(SERVICE_NAME);
  // Use this handle just like a normal function and call it
  ros::service::waitForService(SERVICE_NAME); 
  std_srvs::Empty srv;
  if (gripperClient.call(srv))
  {
    ROS_INFO("Gripper_%d state: %s .", index_, state_.c_str());
  }
}

}





int main(int argc,char **argv)
{
  ros::init(argc,argv,"vaccum_gripper_control_server");
  ros::AsyncSpinner spinner(0); 
  spinner.start();

  ros::NodeHandle nh;
  gripper::gripper vaccum_gripper(&nh);

  ros::waitForShutdown();

  return 0;
}