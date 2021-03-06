/**
 * @file rokae_collision_detection.hpp
 * @brief provide collision detection service
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

#ifndef ROKAE_COLLISION_DETECTION_HPP
#define ROKAE_COLLISION_DETECTION_HPP

#include <ros/ros.h>
#include <string>
#include <moveit/robot_model/aabb.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <rokae_jps_navigation/CheckCollision.h>
#include "JPS_Basis/rokae_jps_basis.hpp"

class self_detector
{
  public:
    /**
     * @brief Construct a new self detector object
     * 
     * @param[in] nodehandle ros node handler
     */
    self_detector(ros::NodeHandle* nodehandle);

    /**
     * @brief To check if current joint configuration is valid or not.
     * 
     * @param[in, out] req CheckCollision.srv request
     * @param[in, out] res CheckCollision.srv response
     * @return true 
     * @return false 
     */
    bool m_collision_detection(rokae_jps_navigation::CheckCollision::Request &req, rokae_jps_navigation::CheckCollision::Response &res);

    //! calculate the Quaternion from Rotation Matrix
    Eigen::Quaternionf m_rotationMatrix2Quaternionf(Eigen::Matrix3f &R);
    
    //! If we calculate the IK, when solution found, IK solver will return a vector who is the infomation of matrix T
    Eigen::Matrix3f m_getRotationMatrix(const std::vector<float> &eef_pose);

    //! get the translation part of the T matrix
    std::vector<float> getTranslation(const std::vector<float> &eef_pose);
  private:
    //! private ros node handler
    ros::NodeHandle nh_;

    //! planning group name
    std::string PLANNING_GROUP;
    
    // planning scene monitor pointer
    planning_scene_monitor::PlanningSceneMonitorPtr psm;

    //! planning scene pointer
    planning_scene::PlanningScenePtr scene;

    //! collision detector service server variable
    ros::ServiceServer detector_service;

    //! visualize aabb
    ros::Publisher pub_aabb;

    //! visualize obb
    ros::Publisher pub_obb;
};


#endif