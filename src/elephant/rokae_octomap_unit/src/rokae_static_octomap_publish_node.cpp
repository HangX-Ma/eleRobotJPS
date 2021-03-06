/**
 * @file rokae_static_octomap_publish_node.cpp
 * @author MContour (m-contour@qq.com)
 * @brief static octomap publisher 
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
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <eigen_conversions/eigen_msg.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// https://www.youtube.com/watch?v=Ib1ISnLlD38
// https://answers.ros.org/question/214386/how-to-publish-a-message-in-a-callback-function/


class octomap_load
{
public:
  octomap_load(ros::NodeHandle* noodehandle);
  void msgs_callback(const octomap_msgs::Octomap::ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher octomap_pub_1;
  ros::Publisher octomap_pub_2;
  ros::Subscriber octomap_sub;
  moveit_msgs::PlanningSceneWorld planning_scene_world;
  moveit_msgs::PlanningScene planning_scene;
};

octomap_load::octomap_load(ros::NodeHandle* noodehandle):nh_(*noodehandle)
{
  // publisher for the planning scene
  // TODO octomap_pub_1 = nh_.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
  octomap_pub_1 = nh_.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
  octomap_pub_2 = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  // subscriber for octomap information
  octomap_sub   = nh_.subscribe<octomap_msgs::Octomap>("octomap_binary", 1, &octomap_load::msgs_callback, this);
  ROS_INFO("rokae_static_octomap_publish_node initialized!");
}

void octomap_load::msgs_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{
  planning_scene_world.octomap.header.frame_id = "world";
  planning_scene_world.octomap.header.stamp = ros::Time::now();
  planning_scene_world.octomap.octomap = *msg;
  planning_scene_world.octomap.origin.position.x = 0.0;
  planning_scene_world.octomap.origin.orientation.w = 1.0;

  planning_scene.world = planning_scene_world;
  planning_scene.is_diff = true;

  ROS_INFO("subscribe successfully");

  octomap_pub_1.publish(planning_scene);
  ros::Duration(0.5).sleep();

  octomap_pub_2.publish(planning_scene);
  ros::Duration(0.5).sleep();

  ROS_INFO_STREAM("octomap successfully published");

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "rokae_static_octomap_publish_node");

  ros::NodeHandle nh;
  octomap_load octomap_unit(&nh);

  ros::Rate loop_rate(0.25);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}