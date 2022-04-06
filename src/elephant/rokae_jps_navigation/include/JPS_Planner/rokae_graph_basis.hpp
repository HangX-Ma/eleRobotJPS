/**
 * @file rokae_graph_search.hpp
 * @brief backend of graph search, implementation JPS
 * @author m-contour
 * @version v8.0
 * @date 2022.03.02
 * @copyright Copyright (c) 2021-2022 m-contour. All rights reserved.
 * @par License:\n This project is released under the Berkerley Software Distribution License.
 */
#ifndef ROKAE_GRAPH_BASIS_HPP
#define ROKAE_GRAPH_BASIS_HPP

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <limits>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <chrono>
#include "JPS_Basis/rokae_jps_basis.hpp"

/**
 * @brief JPS name space, used to determine the <em>JPS Path Planner</em> functions
 * 
 */
namespace JPS
{

/**
 * @brief <tt>enum class</tt> \b HeuristicFunctionType 
 * @details This \c enum will determine the heuristic type that will be used in future planning process
 */
enum class HeuristicFunctionType {
    Manhattan = 0, //!< use manhattan distance cost
    Euclidean = 1, //!< use euclidean distance cost
    Diagonal  = 2  //!< use diagonal distance cost
};    

/**
 * @brief \c Node is a struct for JPS path planning, which will be used to store some necessary information
 * 
 */
struct Node
{
  int dx, dy, dz;                                      //!< direction of expanding, only for JPS default: [0,0,0]
  octomap::OcTreeKey key;                              //!< marker for map point, which can be used to get the point location (x,y,z)
  double f = std::numeric_limits<double>::infinity();  //!< total cost f(n)
  double g = std::numeric_limits<double>::infinity();  //!< cumulative distance g(n)
  double h;                                            //!< distance between current point and goal point h(n), heuristic cost
  
  // override function for `Node` comperation
  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
  bool operator<=(const Node &other) const;

  //! self-defined 3D constructor 
  Node(octomap::OcTreeKey key, int dx = 0, int dy = 0, int dz = 0)
    : dx(dx), dy(dy), dz(dz), key(key)
  {}

  Node(){}
  ~Node(){}
};

/**
 * @brief minimum heap module
 * @details \c CostComparator will be used in \c std::priority_queue
 */ 
struct CostComparator {
  bool operator()(const Node &n1, const Node &n2) const;
};

/**
 * @brief minimum heap module
 * @note This hash function is designed for JPS or A-star implement.
 * @details Hash Function for \c std::unordered_set or \c unordered_map .
 */ 
struct HashFunction {
  bool operator()(const Node &n) const;
};


/**
 * @brief minimum heap module
 * @note This hash function is designed for distance potential field.
 * @details Hash Function for \c std::unordered_set or \c unordered_map .
 */ 
struct DPF_HashFunction {
  bool operator()(const octomap::OcTreeKey &key) const;
};

/**
 * @brief Search and prune neighbors for JPS 3D \n
 * each (dx,dy,dz) contains:
 * - ns: neighbors that are always added
 * - f1: forced neighbors to check
 * - f2: neighbors to add if f1 is forced
 * - nsz: contains different types of moves
 */
struct JPS3DNeib {
    //! The first parament '27' refers to the next movement has 27 directions; and the second parament '3' refers to increment of the next node which will be visited relative to current node, (dx, dy, dz); and the third parament '26' refers to the maximum number of the next node which will be visited. Actually, only when the node at the start point, will the next node has 26 choices. 
    int ns[27][3][26]; 
    //! f1 is used to store the increment of the obstacles needed to be checked at different directions. (obstacles check)
    int f1[27][3][12]; 
    //! f2 is used to store the increment of the probable 'Force Neighbor' node which is relative to the current node at current moving direction of current. node.
    int f2[27][3][12]; 
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced

    //! \c nsz[4][2] is a 2 demensions arrary. The first dimension indicates the four movement types and the second dimension indicates the number of the neighbors needed to be added if current node has at least one forced neighbors
    static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
    JPS3DNeib();
  private:
    /**
     * @brief This is a function used to get all the possible natural neighbors of current direction expansion for current \c Node , which will be processed precisely later.
     * 
     * @param[in] dx The increment relative to the current node on the x-axis
     * @param[in] dy The increment relative to the current node on the y-axis
     * @param[in] dz The increment relative to the current node on the z-axis
     * @param[in] norm1 manhattan value of (dx, dy, dz)
     * @param[in] dev current iteration
     * @param[out] tx natural neighbor increment on x-axis
     * @param[out] ty natural neighbor increment on y-axis
     * @param[out] tz natural neighbor increment on z-axis
     */
    void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);

    /**
     * @brief This is a function used to get all the possible forced neighbors of current direction expansion for current \c Node , which will be processed precisely later.
     * 
     * @param[in] dx The increment relative to the current node on the x-axis
     * @param[in] dy The increment relative to the current node on the y-axis
     * @param[in] dz The increment relative to the current node on the z-axis
     * @param[in] norm1 manhattan value of (dx, dy, dz)
     * @param[in] dev current iteration
     * @param[out] fx check point increment on x-axis
     * @param[out] fy check point increment on y-axis
     * @param[out] fz check point increment on z-axis
     * @param[out] nx forced neighbor point increment on x-axis
     * @param[out] ny forced neighbor point increment on y-axis
     * @param[out] nz forced neighbor point increment on z-axis
     */
    void FNeib( int dx, int dy, int dz, int norm1, int dev, 
        int& fx, int& fy, int& fz,
        int& nx, int& ny, int& nz);
};

/**
 * @brief \<enum class\> PlanningState, used to indicate the JPS planning state
 */
enum class PlanningState
{
  ERROR = -1,     //!< Some errors happen during the planning process.
  PATH_COMPLETE,  //!< Path is complete
  PATH_INCOMPLETE //!< Path is incomplete due to some reasons, such as timeout, surpassing  maximum number of iterations, etc.
};

/**
 * @brief GraphSearch class
 * @details implement Jump Point Search
 */
class GraphSearch
{
  public:
    /**
     * @brief 3D graph search constructor
     *
     * @param[in] xDim map length
     * @param[in] yDim map width
     * @param[in] zDim map height
     * @param[in] eps weight of heuristic, optional, default as 1
     * @param[in] verbose flag for printing debug info, optional, default as False
     */
    GraphSearch(const float &xDim, const float &yDim, const float &zDim, const double &planning_tree_resolution, const octomap::point3d &startCoord, const octomap::point3d &goalCoord, const double &timeout_threshold, const double &eps = 1, const bool &verbose = false);

    /**
     * @brief start 3D planning thread
     * 
     * @param[in] start_coord location of start point with octomap data type
     * @param[in] goal_coord location of start point with octomap data type
     * @param[in] tree octree that stores the information of octomap
     * @param[in] maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
     * @return \c true if plan() successes, or \c false when fails.
     */
    bool plan(const octomap::OcTree &tree, int maxExpand = -1);

    /**
     * @brief This is a function used to do distance potential field planning to drive the original path away from the obstacle
     * 
     * @param[in] tree octree that stores the information of octomap
     * @param[in] region_set [search region node set] User can determine which nodes needed to be taken into account.
     * @param[in] dist_map [distance map] Each key corresponds to a distance cost value, which is greater when the node is closer to the original path.
     * @param[in] potential_weight This is used to calculate the cost for distance map.
     * @return cost of the optimal path of current iteration 
     * @warning TEST NOT PASSED!!!
     */
    double DPF_Plan(octomap::OcTree &tree, std::unordered_set<octomap::OcTreeKey, DPF_HashFunction> &region_set, std::unordered_map<octomap::OcTreeKey, double, DPF_HashFunction> &dist_map, const double &potential_weight);

    //! Get the optimal path
    std::vector<octomap::OcTreeKey> getPathKeys() const;

    //! Get the states in open set
    std::vector<Node> getOpenSet() const;

    //! Get the states in closed set
    std::vector<Node> getCloseSet() const;

    //! Get planning status
    PlanningState getPlanningStatus() const;

    //! set Heuristic type for heuristic function 
    void SetHeuristic(std::string & heu_type);

  private:
    //! heuristic function, calculate heuristic cost from \c key1 to \c key2 
    double getHeu(const octomap::OcTreeKey &key1, const octomap::OcTreeKey &key2, const octomap::OcTree &tree);

    /**
     * @brief Get the Succ objects for A-star distance potential field planning
     * 
     * @param[in] currSucc current \c Node that needs to determine its neighbors
     * @param[in, out] NodeSet Node set stores the neighbors needed to be added
     * @param[in, out] Succ_costSet \c g cost of each neighbor node relative to current node
     * @param[in] tree octree of octomap
     * @param[in] region_set search region
     * @param[in] dist_map distance cost map
     * @param[in] potential_weight potential weight to calculate the \c g cost
     */
    void getSucc(Node& currSucc, std::vector<Node>& NodeSet, std::vector<double>& Succ_costSet, octomap::OcTree &tree, std::unordered_set<octomap::OcTreeKey, DPF_HashFunction> &region_set, std::unordered_map<octomap::OcTreeKey, double, DPF_HashFunction> &dist_map,  const double &potential_weight);

    /**
     * @brief Get the Jps Succ object for JPS planning
     * 
     * @param[in] currSucc current \c Node that needs to determine its neighbors
     * @param[in, out] NodeSet Node set stores the neighbors needed to be added
     * @param[in, out] Succ_costSet \c g cost of each neighbor node relative to current node
     * @param[in] tree octree of octomap
     */
    void getJpsSucc(Node& currSucc, std::vector<Node>& NodeSet, std::vector<double>& Succ_costSet, const octomap::OcTree &tree);
    
    /**
     * @brief calculate if current node has Forced Neighbors
     * 
     * @param[in] dx
     * @param[in] dy
     * @param[in] dz
     * @param[in] key
     * @param[in] tree octomap tree used for map management 
     * @return 'true' if current node has Forced Neighbors, otherwise not.
     */
    bool hasForced(const int &dx, const int &dy, const int &dz, const octomap::OcTreeKey &key, const octomap::OcTree &tree);

    /**
     * @brief recursively call the jump() function to do 'jump'. If a jump satisfies one of the following conditions, this jump can be regarded as an effective jump. Therefore, the 'newKey' value will be the next jump point key value.
     * (1) find the goal node
     * (2) a node has forced neighbors, otherwise this jump has no effect 
     * (3) reach the edge or encounter the obstacles
     * @param[in] currKey current node OctreeKey
     * @param[in] dx increment at direction x
     * @param[in] dy increment at direction y
     * @param[in] dz increment at direction z
     * @param[in, out] newKey next node OctreeKey
     * @param[in] tree Octree
     * @return if current node should be added to OpenList for further process
     * @retval true current node needs to be added to the OpenList
     * @retval false current node should not be added to the OpenList
     */
    bool jump(octomap::OcTreeKey &currKey, const int &dx, const int &dy, const int &dz, octomap::OcTreeKey &newKey, const octomap::OcTree &tree);

    //! Check if (x, y, z) is occupied
    bool isOccupied(octomap::OcTreeNode* currNode, octomap::point3d &point);

    //! Check if (x, y, z) is free
    bool isFree(octomap::OcTreeNode* currNode, octomap::point3d &point);

    /** 
     * @brief Key value to Coordinate value with central bias
     * 
     * @warning (wrong! the bias is correct)
     */
    octomap::point3d keyToCoord_modified(const octomap::OcTreeKey &NodeKey, const octomap::OcTree &tree) = delete;

    /**
     * @brief get the path keys from \c from to \c to
     * 
     * @param[in] from start node
     * @param[in] to   end node
     * @return std::vector\<octomap::OcTreeKey\>: path points's \<vector\> keys 
     */
    std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &from, const Node &to);

    /**
     * @brief update current key to next point key
     * 
     * @param[in] key octree key
     * @param[in] dx  increment at x-axis direction
     * @param[in] dy  increment at y-axis direction
     * @param[in] dz  increment at z-axis direction
     * @return octomap::OcTreeKey 
     */
    octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const int &dx, const int &dy, const int &dz);

    //! calculate euclidean distance from \c p1 to \c p2
    double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

    //! calculate euclidean distance from \c k1 to \c k2
    double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTree &tree);

    /**
     * @brief This is a function used to load paraments from the \c .yaml file
     * 
     * @tparam T parament type
     * @param[in] param_name parament needed to be loaded from the \c .yaml file
     * @param[out] param_dest the destination which will contains the loaded parament
     * @param[in] nh ROS node handler
     * @return \c true if get the parament from the \c .yaml file successfully, otherwise not 
     */
    template <typename T>
    bool parse_param(const std::string &param_name, T &param_dest, const ros::NodeHandle &nh);

    double                          eps_;                      //!< penalty
    bool                            verbose_;                  //!< print information or not
    float                           xDim_, yDim_, zDim_;       //!< x,y,z demension of the focused part of the map
    double                          planning_tree_resolution_; //!< map resolution
    double                          timeout_threshold_;        //!< timeout threshold for planning
    octomap::point3d                goalCoord_;                //!< goal coordination in octomap
    octomap::point3d                startCoord_;               //!< start coordination in octomap
    std::shared_ptr<JPS3DNeib>      jn3d_;                     //!< 3D JPS neighbor classification
    std::vector<octomap::OcTreeKey> path_Keys_;                //!< octree key value set of the path

    std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;  //!< minimum heap
    std::unordered_set<Node, HashFunction>                       open;       //!< Initialize the open list
    std::unordered_set<Node, HashFunction>                       closed;     //!< Initialize the closed list
    std::unordered_map<Node, Node, HashFunction>                 parent_map; //!< first = child, second = parent
    
    //! expansion direction of A-star
    const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                                {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                                {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                                {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};

    HeuristicFunctionType heuristic_function_type = HeuristicFunctionType::Diagonal; //!< heuristic type for A-star and JPS planning
    PlanningState         planning_status_;                                           //!< planning status flag
};
}
#endif