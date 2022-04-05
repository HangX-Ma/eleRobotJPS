/**
 * @file rokae_joint2pose.cpp
 * @brief transfer joints value to pose value
 * @author m-contour
 * @version v8.0
 * @date 2021-2022
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rokae_jps_navigation/joint2pose.h>
#include "rokae_ikfast_wrapper.cpp"
#include "JPS_Basis/rokae_jps_basis.hpp"

//! calculate the Quaternion from Rotation Matrix
Eigen::Quaternionf rotationMatrix2Quaterniond(Eigen::Matrix3f &R)  
{  
    Eigen::Quaternionf q(R);  
    q.normalize();   
    return q;  
}  

//! If we calculate the IK, when solution found, IK solver will return a vector who is the infomation of matrix T.
Eigen::Matrix3f getRotationMatrix(const std::vector<float> &eef_pose, const bool &ifVerbose)
{
  Eigen::Matrix3f matrix;
  int nop = 0;
  for (int row = 0; row < 3; row ++)
  {
    for (int col = 0; col < 3; col++)
    {
      matrix(row, col) = eef_pose.at(row*3+col+nop);
    } 
    nop++; // eef_pose is T = [R,t]
  }
  if (ifVerbose) {
    std::cout << ANSI_COLOR_MAGENTA "Rotation Matrix: " << matrix << ANSI_COLOR_RESET << std::endl;
  }

  return matrix;
}

std::vector<float> getTranslation(const std::vector<float> &eef_pose)
{
  std::vector<float> transl;
  for (int i = 0; i < 3; i++) {
    transl.push_back(eef_pose.at(i*4+3));
  }
  return transl;
}

bool joint2pose(rokae_jps_navigation::joint2pose::Request &req, rokae_jps_navigation::joint2pose::Response &res)
{
  std::shared_ptr<robots::Kinematics> _IKSolver(new robots::Kinematics());
  std::vector<float> tar_joint {req.joint0.data, req.joint1.data, req.joint2.data, req.joint3.data, req.joint4.data, req.joint5.data};
  std::vector<float> tar_eef_pose = _IKSolver->forward(tar_joint);

  Eigen::Matrix3f    eef_rotation = getRotationMatrix(tar_eef_pose, req.ifVerbose);
  Eigen::Quaternionf q            = rotationMatrix2Quaterniond(eef_rotation);
  std::vector<float> transl       = getTranslation(tar_eef_pose);

  if (req.ifVerbose) {
    std::cout << ANSI_COLOR_MAGENTA "input joint configs: (" << req.joint0 << "," << req.joint1 << "," << req.joint2 << "," 
                                              << req.joint3 << "," << req.joint4 << "," << req.joint5 << ")" ANSI_COLOR_RESET << std::endl;

    std::cout << ANSI_COLOR_MAGENTA "convert to pose: (x, y, z, w, i, j, k): (" << transl.at(0) << "," << transl.at(1) << "," << 
                                    transl.at(2) << "," << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ")" ANSI_COLOR_RESET << std::endl;

    // std::vector<float> pose {transl.at(0), transl.at(1), transl.at(2), q.w(), q.x(), q.y(), q.z()};
    // std::vector<float> pose {-0.66, 0.113, 0.30, -0.5, 0.5, 0.5, -0.5};
    // std::vector<float> pose {0.090, 0.113, 0.48, -0.5, 0.5, 0.5, -0.5};
    // std::vector<float> pose {-0.17, 0.11, 0.7, -0.5, 0.5, 0.5, -0.5};
    // std::vector<float> joint = _IKSolver->inverse(pose);
    // if (joint.size() == 0) {
    //   ROS_INFO("ERROR");
    // }
    // ROS_INFO("Inverse result: (%f, %f, %f, %f, %f, %f)", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

    // std::vector<float>  prev_joint_configs {0,0,0,0,0,0};
    // std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> ik_result = _IKSolver->getClosestIK(joint, prev_joint_configs);
    // int min_index = ik_result.second.second;
    // ROS_INFO("Inverse result: (%f, %f, %f, %f, %f, %f)", ik_result.first[min_index][0], ik_result.first[min_index][1], ik_result.first[min_index][2], ik_result.first[min_index][3], ik_result.first[min_index][4], ik_result.first[min_index][5]);

    ROS_INFO(ANSI_COLOR_MAGENTA "done" ANSI_COLOR_RESET);
  }

  return true;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "rokae_joint2pose");
  ros::NodeHandle nh_srv;
  ros::ServiceServer service = nh_srv.advertiseService("/rokae_arm/rokae_joint2pose", joint2pose);
  
  ROS_INFO("[rokae_joint2pose]: Service On. Ready to convert joints to poses.");

  ros::spin();
  return 0;
}