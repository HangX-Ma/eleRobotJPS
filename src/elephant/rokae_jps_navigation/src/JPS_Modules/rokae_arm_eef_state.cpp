/**
 * @file rokae_arm_eef_state.cpp
 * @brief get the manipulator state
 * @author MContour (m-contour@qq.com)
 * @version v8.0
 * @date 2021-2022
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
// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "rokae_jps_navigation/eefState.h"
#include "JPS_Basis/rokae_jps_basis.hpp"

class state_report
{
public:
  //! constructor
  state_report();

  //! get manipulator state
  bool getState_Callback(rokae_jps_navigation::eefState::Request& req, rokae_jps_navigation::eefState::Response& res);

private:
  // robot model pointer
  robot_model::RobotModelConstPtr kinematic_model;
};

//! calculate the Quaternion from Rotation Matrix
Eigen::Quaternionf rotationMatrix2Quaterniond(Eigen::Matrix3f &R)  
{  
    Eigen::Quaternionf q(R);  
    q.normalize();   
    return q;  
}

state_report::state_report()
{  
  // create planning interface by choosing the move group
  moveit::planning_interface::MoveGroupInterface move_group_inter("manipulator");
  this->kinematic_model       = move_group_inter.getRobotModel();
  // get pose reference frame
  const std::string ref_frame = move_group_inter.getPoseReferenceFrame();
  ROS_INFO("robot pose reference frame: %s" ,ref_frame.c_str());
  // get current end effector name
  const std::string curr_eef  = move_group_inter.getEndEffector();
  ROS_INFO("robot current end effector: %s", curr_eef.c_str());
}

bool state_report::getState_Callback(rokae_jps_navigation::eefState::Request& req, rokae_jps_navigation::eefState::Response& res)
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  // [Affine ]https://blog.csdn.net/qq_36013249/article/details/103263143
  const Eigen::Affine3d& manipulator_end = kinematic_state->getGlobalLinkTransform("elerobot_link7");
  Eigen::Matrix3f          rotationMatrix  = static_cast<Eigen::Matrix3f>(manipulator_end.rotation().cast<float>());
  Eigen::Quaternionf       q               = rotationMatrix2Quaterniond(rotationMatrix);

  // if (req.ifVerbose) {
  //   std::cout << ANSI_COLOR_MAGENTA "manipulator end position:" << "(" << manipulator_end.translation().x() << "," << manipulator_end.translation().y() 
  //                                                 << "," << manipulator_end.translation().z() << ")" ANSI_COLOR_RESET << std::endl;
  // }
  res.eef_pose.position.x = manipulator_end.translation().x();
  res.eef_pose.position.y = manipulator_end.translation().y();
  res.eef_pose.position.z = manipulator_end.translation().z();
  res.eef_pose.orientation.x = q.x();
  res.eef_pose.orientation.y = q.y();
  res.eef_pose.orientation.z = q.z();
  res.eef_pose.orientation.w = q.w();

  // ROS_INFO_STREAM(res.eef_pose);

  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::NodeHandle nh;

  state_report eef_state;
  ros::ServiceServer eef_service = nh.advertiseService("/rokae_arm/rokae_eef_state", &state_report::getState_Callback, &eef_state);
  
  ROS_INFO_STREAM("[rokae_arm_eef_state]: end_effector state service ready.");
  ros::spin();
  
  return 0;
} 
