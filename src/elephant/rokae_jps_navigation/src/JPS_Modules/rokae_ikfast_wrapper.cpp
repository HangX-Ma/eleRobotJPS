/*
 * THIS IS A MODIFIED VERSION OF THE FOLLOWING:
 * 
 * IKFast Demo
 * 
 * Shows how to calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 * 
 * 
 * To compile, run:
 * g++ -lstdc++ -llapack -o compute ikfastdemo.cpp -lrt
 * (need to link with 'rt' for gettime(), it must come after the source file name)
 *
 * 
 * Tested with Ubuntu 11.10 (Oneiric)
 * IKFast54 from OpenRAVE 0.6.0
 * IKFast56/61 from OpenRave 0.8.2
 *
 * Author: David Butterworth, KAIST
 *         Based on code by Rosen Diankov
 * Date: November 2012
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 61
#include "rokae_arm_manipulator_ikfast_solver.cpp"

//#define IK_VERSION 56
//#include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 54
//#include "output_ikfast54.cpp"


//----------------------------------------------------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <vector>
#include <math.h>
#include <float.h>
#include <ros/ros.h>

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif

namespace robots { 
    /**
     * @date 2021.11.22
     * @author contour
     * @version v1.0
     * @brief We need to check the joint configs from the 'ikCompute' function is valid, which means, the joint configs for target pose
     *        needs to within the joint limits and to be effective.
     * @param solution_num_ solution number
     * @param effective_index effective solution groups value
     * @param rokae_upper_limits joint upper limits
     * @param rokae_lower_limits joint lower limits
     * @param divided_joint_configs divdie the solutions, which in one vector container, to several groups according to the joint number
    */
    class Kinematics { 
      public: 
        int num_of_joints, num_free_parameters;
        int solution_num_; 
        std::vector<int> effective_index;
        std::vector<float> rokae_upper_limits;
        std::vector<float> rokae_lower_limits;
        std::vector<std::vector<float>> divided_joint_configs;
        Kinematics(); 
        ~Kinematics(); 
        std::vector<float> forward(std::vector<float> joint_config);
        std::vector<float> inverse(std::vector<float> ee_pose);
        int solution_num(const std::vector<float> joint_configs);
        std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> getClosestIK(const std::vector<float> joint_configs, const std::vector<float> previous_solution);
        void GetUpperLimits();
        void GetLowerLimits();
        bool CheckBound(const std::vector<float> joint_configs);
    }; 

    // constructor
    Kinematics::Kinematics() { 
            GetUpperLimits();
            GetLowerLimits();
        #if IK_VERSION > 54
            // for IKFast 56,61
            num_of_joints = GetNumJoints();
            num_free_parameters = GetNumFreeParameters();
        #else
            // for IKFast 54
            num_of_joints = getNumJoints();
            num_free_parameters = getNumFreeParameters();
        #endif
    }

    // destructor
    Kinematics::~Kinematics() { } 

    // forward computation
    std::vector<float> Kinematics::forward(std::vector<float> joint_config) {
        IKREAL_TYPE eerot[9],eetrans[3];
        std::vector<float> ee_pose;

        if( (int)joint_config.size() != num_of_joints ) {
            printf("\nError: (forward kinematics) expects vector of %d values describing joint angles (in radians).\n\n", num_of_joints);
            return ee_pose;
        }

        // Put input joint values into array
        IKREAL_TYPE joints[num_of_joints];
        for (int i=0; i<num_of_joints; i++)
        {
            joints[i] = joint_config[i];
        }

#if IK_VERSION > 54
        // for IKFast 56,61
        ComputeFk(joints, eetrans, eerot); // void return
#else
        // for IKFast 54
        fk(joints, eetrans, eerot); // void return
#endif
        for (int i=0; i<3; i++) {
            ee_pose.push_back(eerot[i*3+0]);
            ee_pose.push_back(eerot[i*3+1]);
            ee_pose.push_back(eerot[i*3+2]);
            ee_pose.push_back(eetrans[i]);
        }

        return ee_pose;
    }

    // inverse computation
    std::vector<float> Kinematics::inverse(std::vector<float> ee_pose) {
        IKREAL_TYPE eerot[9],eetrans[3];
        std::vector<float> joint_configs;

        if( ee_pose.size() == 7 )  // ik, given translation vector and quaternion pose
        {
#if IK_VERSION > 54
            // for IKFast 56,61
            IkSolutionList<IKREAL_TYPE> solutions;
#else
            // for IKFast 54
            std::vector<IKSolution> vsolutions;
#endif
            std::vector<IKREAL_TYPE> vfree(num_free_parameters);

            eetrans[0] = ee_pose[0];
            eetrans[1] = ee_pose[1];
            eetrans[2] = ee_pose[2];

            // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
            // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
            double qw = ee_pose[3];
            double qx = ee_pose[4];
            double qy = ee_pose[5];
            double qz = ee_pose[6];
            const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;
            eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
            eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
            eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

            // For debugging, output the matrix
            /*
            for (unsigned char i=0; i<=8; i++)
            {   // detect -0.0 and replace with 0.0
                if ( ((int&)(eerot[i]) & 0xFFFFFFFF) == 0) eerot[i] = 0.0;
            }
            printf("     Rotation     %f   %f   %f  \n", eerot[0], eerot[1], eerot[2] );
            printf("                  %f   %f   %f  \n", eerot[3], eerot[4], eerot[5] );
            printf("                  %f   %f   %f  \n", eerot[6], eerot[7], eerot[8] );
            printf("\n");
            */

            // for(std::size_t i = 0; i < vfree.size(); ++i)
            //     vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
            // for IKFast 56,61
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
            if( !bSuccess ) {
                fprintf(stderr,"Error: (inverse kinematics) failed to get ik solution\n");
                return joint_configs;
            }

#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            // printf("Found %d ik solutions:\n", num_of_solutions ); 

            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
            for(std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
                // for IKFast 56,61
                const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                int this_sol_free_params = (int)sol.GetFree().size(); 
#else
                // for IKFast 54
                int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
                // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
                // for IKFast 56,61
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#else
                // for IKFast 54
                vsolutions[i].GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#endif

                for( std::size_t j = 0; j < solvalues.size(); ++j)
                    joint_configs.push_back(solvalues[j]);
                    // printf("%.15f, ", solvalues[j]);
                // printf("\n");
            }

        } 
        else if( ee_pose.size() == 12 )  // ik, given rotation-translation matrix
        {
#if IK_VERSION > 54
            // for IKFast 56,61
            IkSolutionList<IKREAL_TYPE> solutions;
#else
            // for IKFast 54
            std::vector<IKSolution> vsolutions;
#endif
            std::vector<IKREAL_TYPE> vfree(num_free_parameters);

            eerot[0] = ee_pose[0]; eerot[1] = ee_pose[1]; eerot[2] = ee_pose[2];  eetrans[0] = ee_pose[3];
            eerot[3] = ee_pose[4]; eerot[4] = ee_pose[5]; eerot[5] = ee_pose[6];  eetrans[1] = ee_pose[7];
            eerot[6] = ee_pose[8]; eerot[7] = ee_pose[9]; eerot[8] = ee_pose[10]; eetrans[2] = ee_pose[11];
            // for(std::size_t i = 0; i < vfree.size(); ++i)
            //     vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
            // for IKFast 56,61
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
            if( !bSuccess ) {
                fprintf(stderr,"Error: (inverse kinematics) failed to get ik solution\n");
                return joint_configs;
            }

#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            // printf("Found %d ik solutions:\n", num_of_solutions ); 

            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
            for(std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
                // for IKFast 56,61
                const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                int this_sol_free_params = (int)sol.GetFree().size(); 
#else
                // for IKFast 54
                int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
                // printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
                // for IKFast 56,61
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#else
                // for IKFast 54
                vsolutions[i].GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#endif
                for( int j = 0; j < (int)solvalues.size(); ++j)
                    joint_configs.push_back(solvalues[j]);
                //     printf("%.15f, ", solvalues[j]);
                // printf("\n");
            }

        }
        else {
            printf("\nError: (inverse kinematics) please specify transformation of end effector with one of the following formats:\n"
                   "    1) A vector of 7 values: a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k)\n"
                   "    2) A (row-major) vector of 12 values: a 3x4 rigid transformation matrix with a 3x3 rotation R (rXX), and a 3x1 translation (tX)\n\n");
            return joint_configs;

        }

        return joint_configs;
    }

    // get solution number
    int Kinematics::solution_num(const std::vector<float> joint_configs)
    {
      return static_cast<int>(joint_configs.size()/num_of_joints);
    } // get solution number

    // joint upper limits for work
    void Kinematics::GetUpperLimits()
    {
      rokae_upper_limits.push_back(0.008226646);
      rokae_upper_limits.push_back(2.268428027);
      rokae_upper_limits.push_back(1.133964013);
      rokae_upper_limits.push_back(0.008726646);
      rokae_upper_limits.push_back(1.594395102);
      rokae_upper_limits.push_back(6.282685307);
      // ROS_INFO_STREAM("rokae_upper_limits: " << rokae_upper_limits.at(0) << ", " << rokae_upper_limits.at(1) << ", " << rokae_upper_limits.at(2)
      //                                       << ", " << rokae_upper_limits.at(3) << ", " << rokae_upper_limits.at(4) << ", " << rokae_upper_limits.at(5));
    }

    // joint lower limits for work
    void Kinematics::GetLowerLimits()
    {
      rokae_lower_limits.push_back(-0.008226646);
      rokae_lower_limits.push_back(-1.675016081);
      rokae_lower_limits.push_back(-3.402892041);
      rokae_lower_limits.push_back(-0.008726646);
      rokae_lower_limits.push_back(-1.594395102);
      rokae_lower_limits.push_back(-6.282685307);
      // ROS_INFO_STREAM("rokae_upper_limits: " << rokae_lower_limits.at(0) << ", " << rokae_lower_limits.at(1) << ", " << rokae_lower_limits.at(2)
      //                                       << ", " << rokae_lower_limits.at(3) << ", " << rokae_lower_limits.at(4) << ", " << rokae_lower_limits.at(5));
    }

    // check if the solution obey the rules whether or not
    bool Kinematics::CheckBound(const std::vector<float> joint_configs)
    {
      solution_num_ = solution_num(joint_configs);

      if (solution_num_ == 0)
      {
        // TODO ROS_WARN_STREAM("ikfast solver find no solution for target pose.");
        return false;
      }

      // TODO ROS_INFO_STREAM("ikfast solution number: " << std::to_string(solution_num_));


      float curr_val;
      std::string ros_info;

      for (int i = 0; i < solution_num_; i++)
      {
        // add a new row
        divided_joint_configs.push_back(std::vector<float>());

        for (int j = 0; j < num_of_joints; j++)
        {
          curr_val = joint_configs.at(i * num_of_joints + j);
          // between rokae_upper_limits and rokae_lower_limits needs to be taken into account
          if (curr_val <= rokae_upper_limits.at(j) && curr_val >= rokae_lower_limits.at(j))
          {
            divided_joint_configs[i].push_back(curr_val);
          }

          if (j == 2)
          {
            if(curr_val  > rokae_upper_limits.at(j))
            {
              float lower_tmp = curr_val - 2*M_PI;
              if (lower_tmp >= rokae_lower_limits.at(j))
              {
              divided_joint_configs[i].push_back(lower_tmp);
              }
            }
          }
        }

        if ((int)divided_joint_configs[i].size() == num_of_joints)
        {
          effective_index.push_back(i);
          ros_info += std::to_string(i);
          ros_info += " ";
        }
      }

      if ((int)effective_index.size() == 0)
      {
        // TODO ROS_WARN_STREAM("current pose has no effective joint values to reach.");
        return false;
      }

      // TODO ROS_INFO_STREAM("effective solution index: " << ros_info); 

      return true;
    } // store the joint configs in 2D vector[solution_number][joint_number]


    // get the best suitable solution for target pose
    std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> Kinematics::getClosestIK(const std::vector<float> joint_configs, const std::vector<float> previous_solution)
    {
      double PENALTY_JOINT0 = 1.0;
      double PENALTY_JOINT1 = 100.0;
      double PENALTY_JOINT2 = 100.0;
      double PENALTY_JOINT3 = 1.0;
      double PENALTY_JOINT4 = 10.0;
      double PENALTY_JOINT5 = 5.0;
      double min_err = FLT_MAX;
      int min_index;

      if ((int)previous_solution.size() != num_of_joints)
      {
        ROS_ERROR_STREAM("previous joint number is not equal to the robot joint number");
        std::pair<std::vector<int>, int> sol_result (std::vector<int>(), -1);
        std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> result_paired(std::vector<std::vector<float>>(), sol_result);
        return result_paired;
      }

      if(!CheckBound(joint_configs))
      {
        // TODO ROS_WARN_STREAM("Policy is needed to determine effective target pose.");
        std::pair<std::vector<int>, int> sol_result (std::vector<int>(), -1);
        std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> result_paired(std::vector<std::vector<float>>(), sol_result);
        return result_paired;
      } // check solution bound

      if (effective_index.size() == 0)
      {
        ROS_INFO_STREAM("No effective solution.");
        std::pair<std::vector<int>, int> sol_result (std::vector<int>(), -1);
        std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> result_paired(std::vector<std::vector<float>>(), sol_result);
        return result_paired;
      }


      for (int i = 0; i < (int)effective_index.size(); i++)
      {
        double err_tmp = 0; 
        int eef_index = effective_index.at(i);
        // TODO
        // ROS_INFO_STREAM("JOINT CONFIGS:" << eef_index);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][0]);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][1]);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][2]);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][3]);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][4]);
        // ROS_INFO_STREAM(divided_joint_configs[eef_index][5]);
        // ROS_INFO_STREAM("PREV JOINT:");
        // ROS_INFO_STREAM(previous_solution[0]);
        // ROS_INFO_STREAM(previous_solution[1]);
        // ROS_INFO_STREAM(previous_solution[2]);
        // ROS_INFO_STREAM(previous_solution[3]);
        // ROS_INFO_STREAM(previous_solution[4]);
        // ROS_INFO_STREAM(previous_solution[5]);

        err_tmp += (double)(divided_joint_configs[eef_index][0] - previous_solution[0]) * (double)(divided_joint_configs[eef_index][0] - previous_solution[0]) * PENALTY_JOINT0;
        err_tmp += (double)(divided_joint_configs[eef_index][1] - previous_solution[1]) * (double)(divided_joint_configs[eef_index][1] - previous_solution[1]) * PENALTY_JOINT1;
        err_tmp += (double)(divided_joint_configs[eef_index][2] - previous_solution[2]) * (double)(divided_joint_configs[eef_index][2] - previous_solution[2]) * PENALTY_JOINT2;
        err_tmp += (double)(divided_joint_configs[eef_index][3] - previous_solution[3]) * (double)(divided_joint_configs[eef_index][3] - previous_solution[3]) * PENALTY_JOINT3;
        err_tmp += (double)(divided_joint_configs[eef_index][4] - previous_solution[4]) * (double)(divided_joint_configs[eef_index][4] - previous_solution[4]) * PENALTY_JOINT4;
        err_tmp += (double)(divided_joint_configs[eef_index][5] - previous_solution[5]) * (double)(divided_joint_configs[eef_index][5] - previous_solution[5]) * PENALTY_JOINT5;

        if (min_err > err_tmp)
        {
          min_index = eef_index;
          min_err = err_tmp;
        }
      } // calculate the joints value which are closet to the previous joints
      // TODO
      // ROS_INFO_STREAM("best solution found, index" << "[" << std::to_string(min_index) << "]"); 
      std::pair<std::vector<int>, int> sol_result (effective_index, min_index);
      std::pair<std::vector<std::vector<float>>, std::pair<std::vector<int>, int>> result_paired(divided_joint_configs, sol_result);
      return result_paired;
    }

}





#ifndef IKFAST_NO_MAIN
int main(int argc, char** argv)
{
    IKREAL_TYPE eerot[9],eetrans[3];

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif
 
    std::string cmd;
    if (argv[1]) cmd = argv[1];

    //printf("command: %s \n\n", cmd.c_str() );

    if (cmd.compare("ik") == 0) // ik mode
    {
    } // endif ik mode

    else if (cmd.compare("fk") == 0) // fk mode
    {
        

    }
    else if (cmd.compare("iktiming") == 0) // generate random ik and check time performance
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming \n\n"
                   "         For fixed number of iterations, generates random joint angles, then  \n"
                   "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        srand( (unsigned)time(0) ); // seed random number generator
        float min = -3.14;
        float max = 3.14;
        float rnd;

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            // Measure avg time for whole process
            //clock_gettime(CLOCK_REALTIME, &start_time); 

            // Put random joint values into array
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                float rnd = (float)rand() / (float)RAND_MAX;
                joints[i] = min + rnd * (max - min);
            }
            /*
            printf("Joint angles:  ");
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                printf("%f  ", joints[i] );
            }
            printf("\n");
            */

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeFk(joints, eetrans, eerot); // void return
#else
            // for IKFast 54
            fk(joints, eetrans, eerot); // void return
#endif

            // Measure avg time for IK
            clock_gettime(CLOCK_REALTIME, &start_time);
#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;
  
    }
    else if (cmd.compare("iktiming2") == 0) // for fixed joint values, check time performance of ik
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming2 \n\n"
                   "         For fixed number of iterations, with one set of joint variables, this  \n"
                   "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        // fixed rotation-translation matrix corresponding to an unusual robot pose
        eerot[0] = 0.002569;  eerot[1] = -0.658044;  eerot[2] = -0.752975;  eetrans[0] = 0.121937;
        eerot[3] = 0.001347;  eerot[4] = -0.752975;  eerot[5] = 0.658048;  eetrans[1] = -0.276022;
        eerot[6] = -0.999996; eerot[7] = -0.002705; eerot[8] = -0.001047; eetrans[2] = 0.005685;

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            clock_gettime(CLOCK_REALTIME, &start_time);

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;

    } else {
        printf("\n"
               "Usage: \n\n");
        printf("         ./compute fk j0 j1 ... j%d \n\n"
               "         Returns the forward kinematic solution given the joint angles (in radians). \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute ik  t0 t1 t2  qw qi qj qk  free0 ... \n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf(" \n"
               "         ./compute ik  r00 r01 r02 t0  r10 r11 r12 t1  r20 r21 r22 t2  free0 ...\n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x3 rotation R (rXX), and a 3x1 translation (tX). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf("\n"
               "         ./compute iktiming \n\n"
               "         For fixed number of iterations, generates random joint angles, then \n"
               "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute iktiming2 \n\n"
               "         For fixed number of iterations, with one set of joint variables, this \n"
               "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );

        return 1;
    }

    return 0;
}
#endif


float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}
