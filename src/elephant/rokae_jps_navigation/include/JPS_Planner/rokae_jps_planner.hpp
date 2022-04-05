/**
 * @file rokae_jps_planner.hpp
 * @brief JPS Planner
 * @details This file include the <tt>rokae_graph_basis.hpp</tt> to realize the JPS search function
 * @author m-contour
 * @version v8.0
 * @date 2022.03.02
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ROKAE_JPS_PLANNER_HPP
#define ROKAE_JPS_PLANNER_HPP

#include <octomap_msgs/conversions.h>
#include <moveit/planning_interface/planning_interface.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "rokae_jps_navigation/eefState.h"
#include "rokae_jps_navigation/Goto.h"
#include "rokae_jps_navigation/CheckCollision.h"
#include "rokae_arm_toppra/ToppRa_srv.h"

#include "JPS_Planner/rokae_graph_basis.hpp"

/**
 * @brief using <enum class> type to indicate planning state
 * 
 * @details The planning function will store the "PlanningState::XXXX" in the "planning_status"
 */
enum class PlanningState
{
  ERROR = -1,      //!< some errors occur in planning process
  NEED_REPLANNING, //!< the given path contains some points that are obstacles or the manipulator can not reach
  NEED_OPTIMAL,    //!< the given path need to be optimized to be driven away from the obstacles
  SUCCESS,         //!< planning success, generating the 'toppra' trajectory then storing the value in the private variables
  FAILURE,         //!< planning fails
};    


/**
 * @brief JPSPlanner class is used to integrate the JPS planner function, which is realized in the <tt>rokae_grapth_basis.hpp</tt>. 
 */
class JPSPlanner
{
  public:
    JPSPlanner(ros::NodeHandle* nodehandle);
    
    //! end effector pose clinet
    geometry_msgs::Pose eef_state_client();

    //! toppra client
    bool toppra_client();
    
    /**
     * @brief collision detection client, if a certain node is detected as an obstacle, this function will change that node value in octree 
     * 
     * @param waypoints path points
     * @param[in, out] tree octree of octomap
     */
    void collision_detection_client(std::vector<octomap::point3d> &waypoints, std::shared_ptr<octomap::OcTree> &tree);

    //! check pose avalability
    bool tarPoseAvalability_client(std::vector<geometry_msgs::Pose> &tarPoseSet);

    //! get the octomap information from the octomap server
    void octomapCallback(const octomap_msgs::Octomap &msg);

    //! action operation server
    bool gotoCallback(rokae_jps_navigation::Goto::Request &req, rokae_jps_navigation::Goto::Response &res);
    
    //! resample way points with desired pose
    void resamplePath(std::vector<octomap::point3d> &waypoints, std::vector<geometry_msgs::Pose> &tarPoints_pose_msg_local);

    //! generate junction goal 
    void genJunction();
    /**
     * @brief modified target point to satisfied the coordinate value under resolution limitation
     * 
     * @param[in] point target point needs to be modified
     * @return modified octomap::point3d
     */
    octomap::point3d suit_coordinate(octomap::point3d &point);

    //! Get the optimized path
    std::vector<octomap::point3d> getPath() const;

    //! Get the raw path
    std::vector<octomap::point3d> getRawPath() const;

    //! visualize a specific point
    void visualizePoint(const octomap::point3d &point, int msg_id);

    //! visualize a specific voxel
    void visualizeVoxel(const octomap::point3d &point);

    //! visualize the octomap tree
    void visualizeTree(const octomap::OcTree &tree);

    //! visualize the searched way points
    void visualizePath(const std::vector<octomap::point3d> &waypoints);

    /**
     * @brief visualize the expansion process
     * 
     * @param[in] open open set of graph planning
     * @param[in] closed closed set of graph planning
     * @param[in] tree octree of octomap
     */
    void visualizeExpansions(const std::vector<JPS::Node> &open, const std::vector<JPS::Node> &closed, const octomap::OcTree &tree);

  public:
    /**
     * @brief Set the Search region object
     * 
     * @param[in] path waypoints of the optimal path, stored in std::vector<>
     * @param[in] search_radius search radius of node distance around the given path
     * @param[in] tree Octree of octomap
     */
    void setSearchRegion(std::vector<octomap::point3d> &path, const int search_radius, const int potential_radius, octomap::OcTree &tree);

    //! set search region for distance potential field
    void setSearchRegion(std::vector<octomap::OcTreeKey> &keys, const int search_radius, const int potential_radius, octomap::OcTree &tree);

    /**
     * @brief interatively compute a path that can drive the original path away from the obstacle 
     * 
     * @param[in] max_iteration max iteration times of the computing process
     * @param[in] tree octree of octomap
     * @param[in] planner planner that supports JPS search and A-star distance potential field computation
     * @return planning result information
     * @retval true optimal process has been done successfully
     * @retval false computation error occurs
     */
    bool iterativeComputePath(int max_iteration, octomap::OcTree &tree);

    //! iterative path optimization process
    bool plan(octomap::OcTree &tree);

    //! get planning stauts
    PlanningState getPlanningStatus() const;

    //! get toppra position result
    std::vector<double> getToppraPos() const;

    //! get toppra velocity result
    std::vector<double> getToppraVel() const;

    //! get toppra acceleration result
    std::vector<double> getToppraAcc() const;

    //! get toppra time result
    std::vector<double> getToppraT() const;

  private:
    // state indicator
    bool getting_octomap_ = false;   //!< indicator for octomap state
    bool is_initialized_  = false;   //!< indicator for initialization process state
    bool show_unoccupied_ = false;   //!< show unoccupied voxels of the octree 
    bool back_move_       = false;   //!< move manipulator back to the beginning position
    bool check_status_    = false;   //!< a flag indicates if the user can set the initial joint configs or not
    bool planner_verbose_ = false;   //!< print information or not
    bool debug_verbose_   = false;   //!< print information or not

    ros::NodeHandle                  nh_;                       //!< ros node handler
    std::shared_ptr<octomap::OcTree> octree_;                   //!< octree smart pointer
    std::string                      parent_frame_;             //!< parent frame value used in msg transfer
    geometry_msgs::Pose              prevPose_;                 //!< previous planning pose
    std::vector<geometry_msgs::Pose> tarPose_buffer_;          //!< buffer storing the goal pose
    octomap::point3d                 prevGoal_;                 //!< previous planning goal
    octomap::point3d                 currGoal_;                 //!< current planning goal
    int                              currGoal_id_;              //!< goal id for identification
    std::vector<octomap::point3d>    tarPoints_bufferIn;        //!< multiple planning goals buffer
    std::vector<Eigen::Vector4f>     tarPoints_pose_;           //!< multiple planning poses buffer, using \c currGoal_id_ to locate the content 
    std::vector<geometry_msgs::Pose> tarPoints_pose_msg_;       //!< pose msg buffer  
    std::vector<std::vector<float>>  joint_configs_;            //!< joints value output buffer

    // collision detection
    std::vector<float>  prev_joint_configs_ = std::vector<float>(6); //!< previous joint angle values
    std::vector<float>  curr_joint_configs_ = std::vector<float>(6); //!< current joint angle values

    // toppra module
    std::vector<double> toppra_pos_;                              //!< \c position values from toppra trajectory planner
    std::vector<double> toppra_vel_;                              //!< \c velocity values from toppra trajectory planner
    std::vector<double> toppra_acc_;                              //!< \c acceleration values from toppra trajectory planner
    std::vector<double> toppra_t_;                                //!< \c time values from toppra trajectory planner

    // visualization params
    double   tree_points_scale_;                                  //!< scale factor of tree points
    double   expansions_points_scale_;                            //!< scale factor of expansion points
    double   path_points_scale_;                                  //!< scale factor of way points
    double   point_scale_;                                        //!< scale factor of a specific point
    double   voxel_scale_;                                        //!< scale factor of a specific voxel
    uint32_t obs_iter_id_;                                        //!< visualization id for obstacle voxel
    // visualization module
    /**
     * @brief generate color for marker 'POINTS'
     * 
     * @param[in] r \b red red value
     * @param[in] g \b green green value
     * @param[in] b \b blue blue value
     * @param[in] a \b alpha transparent if 'a' is equal to zero
     * @return std_msgs::ColorRGBA 
     *
     */
    std_msgs::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

    // publishers 
    ros::Publisher     point_publisher_;            //!< point ros pulisher for visualization
    ros::Publisher     voxel_publisher_;            //!< voxel ros pulisher for visualization
    ros::Publisher     binary_tree_publisher_;      //!< octomap binary tree ros publisher for visualization
    ros::Publisher     path_publisher_;             //!< JPS path ros publisher for visualization
    ros::Publisher     expansion_publisher_;        //!< expension progress ros publisher for visualization

    // subscriber
    ros::Subscriber    octomap_subscriber_;         //!< subscribe the octomap server to get the map information

    // service server
    ros::ServiceServer goto_service_;               //!< provide the planning service

    // service client
    ros::ServiceClient eef_state_client_;           //!< end effector state requirement client
    ros::ServiceClient collision_detection_client_; //!< collision detection client
    ros::ServiceClient toppra_client_;              //!< toppra trajectory planner client

    /**
     * @brief load paraments from .yaml file 
     * 
     * @tparam T type
     * @param param_name parament name
     * @param param_dest parament container
     * @return \c true parament loads successfully; \c false fails
     */
    template <typename T>
    bool parse_param(const std::string &param_name, T &param_dest);

  private:
    //! get the new key values of the neighbors
    octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction);

    //! Check if (x, y, z) is occupied
    bool isOccupied(octomap::OcTreeNode* currNode);

    //! Check if (x, y, z) is free
    bool isFree(octomap::OcTreeNode* currNode);

    //! Check if current coords is outside of the searching region
    bool isOutside(const octomap::point3d point);

    /**
     * @brief get the neighbors'increment at certain direction
     * 
     * @param[in] radius neighbors attaining radius 
     * @param[in, out] expension_dirs expension increments set at different directions 
     */
    void getDirSet(const int radius, std::vector<std::vector<int>> &expension_dirs);

    //! set current node property to obstacle
    bool setCurrNodeObs(octomap::OcTreeKey &key, std::shared_ptr<octomap::OcTree> &tree);

    //! User can set initial joints value for manipulator before the collision detection client initiliazed.
    bool setInitialJoints(float &&j1, float &&j2, float &&j3, float &&j4, float &&j5, float &&j6);

    /**
     * @brief This function is used to filter waypoints, removing redundant points on the same line. If two waypoints of the waypoint set go through obstacle or unknown cell, we need to record the waypoints. Otherwise, we can ignore those points. 
     * 
     * @param[in] waypoints waypoints from search
     * @param[in] tree octree
     * @return std::vector<octomap::point3d> 
     */
    std::vector<octomap::point3d> removeLinePts(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

    /**
     * @brief Remove some corner waypoints 
     * 
     * @details This function reduce half of the waypoints once you call it.(in a triangle, the third_edge < first_edge + second_edge)
     * @param waypoints the path points that needs to be optimized
     * @param tree octree of octomap
     * @return optimal path points
     */
    std::vector<octomap::point3d> removeCornerPts(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

    //! keys to coordinate <vector> type
    std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree); 

    //! key to coordinate type
    octomap::point3d keyToCoord_modified(octomap::OcTreeKey &key, octomap::OcTree &tree);

    //! key to coordinate <vector> type
    std::vector<octomap::OcTreeKey> coordsToKeys(std::vector<octomap::point3d> &path, octomap::OcTree &tree); 

    /**
     * @brief find a path between two points straightly
     * 
     * @param[in] p1 origin point
     * @param[in] p2 goal point
     * @param[in] tree octomap tree
     * @return \c true successful; \c false failed
     */
    bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

    std::unordered_set<octomap::OcTreeKey, JPS::DPF_HashFunction>         region_set; //!< search region node set
    std::unordered_map<octomap::OcTreeKey, double, JPS::DPF_HashFunction> dist_map;   //!< distance potential field cost map, in which the cost correspond to relative nodes 

    //! Check if current point is avaliable
    bool checkAvailability(octomap::point3d &point);

  private:
    float                             obs_val_          {1.0};   //!< obstacle occupancy value
    int                               max_iteration_    {10};    //!< maximum iteration times of optimal process
    int                               JPS_max_iteration_{-1};    //!< maximum iteration times of optimal process
    int                               mask_gain_        {100};   //!< mask gain for tunnel voxel
    double                            eps_              {1.0};   //!< heuristic gain
    double                            potential_weight_ {0.1};   //!< potential weight
    int                               potential_radius_ {2};     //!< potential field radius
    int                               search_radius_    {1};     //!< potential field radius
    float                             xDim_, yDim_, zDim_;       //!< maximum length of each coordinate in 3D space for route planning
    std::string                       heu_type_;                 //!< heuristic type
    double                            planning_timeout_{300};    //!< planning process timeout
    double                            JPS_timeout_{30000};       //!< JPS planning timeout
    double                            planning_tree_resolution_; //!< octomap resolution
    double                            path_cost_;                //!< used in interatively computing path
    std::vector<octomap::point3d>     path_;                     //!< processed path
    std::vector<octomap::point3d>     raw_path_;                 //!< original path
    std::vector<octomap::point3d>     optimal_path_;             //!< optimal path generated from processed path
    std::vector<octomap::OcTreeKey>   dist_path_keys_;           //!< distance property contained path
    PlanningState                     planning_status_;          //!< planning status flag

    std::shared_ptr<JPS::GraphSearch> JPS_BasePtr;      //!< implement of JPS planner

};


#endif
