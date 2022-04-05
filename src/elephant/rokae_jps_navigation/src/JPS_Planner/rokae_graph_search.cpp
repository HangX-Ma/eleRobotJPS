#include "JPS_Planner/rokae_graph_basis.hpp"

using namespace JPS;
bool Node::operator==(const Node &other) const {
  return key == other.key;
}

bool Node::operator!=(const Node &other) const {
  return key != other.key;
}

bool Node::operator<(const Node &other) const {
  if (f == other.f) {
    return h < other.h;
  }

  return f < other.f;
}

bool Node::operator<=(const Node &other) const {
  if (f == other.f) {
    return h <= other.h;
  }

  return f <= other.f;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {
    if (n1.f == n2.f) {
      return n1.h > n2.h;
    }
    return n1.f > n2.f;
};


bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

bool DPF_HashFunction::operator()(const octomap::OcTreeKey &key) const {
  using std::hash;
  return ((hash<int>()(key.k[0]) ^ (hash<int>()(key.k[1]) << 1)) >> 1) ^ (hash<int>()(key.k[2]) << 1);
}

GraphSearch::GraphSearch(const float &xDim, const float &yDim, const float &zDim, const double &planning_tree_resolution, const octomap::point3d &startCoord, const octomap::point3d &goalCoord, const double &timeout_threshold, const double &eps/* = 1*/, const bool &verbose/* = false*/)
{
  this->xDim_                     = xDim;
  this->yDim_                     = yDim;
  this->zDim_                     = zDim;
  this->planning_tree_resolution_ = planning_tree_resolution;
  this->startCoord_               = startCoord;
  this->goalCoord_                = goalCoord;
  this->jn3d_                     = std::make_shared<JPS3DNeib>();
  this->eps_                      = eps;
  this->verbose_                  = verbose;
  this->timeout_threshold_        = timeout_threshold;
}

bool GraphSearch::plan(const octomap::OcTree &tree, int maxExpand /* = -1*/)
{   
  if (verbose_) {
    printf(ANSI_COLOR_CYAN "Planning region dimension: x=%f, y=%f, z=%f" ANSI_COLOR_RESET "\n", xDim_, yDim_, zDim_);
  }
  // the clock with the shortest tick period available
  auto time_start = std::chrono::high_resolution_clock::now(); 
  // clear previous value
  open.clear();
  closed.clear();
  parent_map.clear();
  std::priority_queue<Node, std::vector<Node>, CostComparator> null_heap;
  swap(open_heap, null_heap);
  path_Keys_.clear();

  // octreeKey of 'start_coord' and 'goal_coord'
  // [Note]: coordToKey() function does not lead to any basis but keyToCoord() does.
  octomap::OcTreeKey startKey = tree.coordToKey(startCoord_);
  octomap::OcTreeKey goalKey  = tree.coordToKey(goalCoord_);

  // create the start Node
  Node startNode(startKey, 0, 0, 0);
  startNode.g   = 0;
  startNode.h   = getHeu(startKey, goalKey, tree);
  startNode.f   = startNode.g + startNode.h;

  // timeout triggers
  Node bestNode = startNode;

  // insert start node
  open_heap.push(startNode);
  open.insert(startNode);

  int expand_iteration = 0;
  Node currNode;
  // this is the main loop
  while (!open.empty() && ros::ok()) 
  {
    expand_iteration++;
    // get element with smallest cost (we initially built a minimum heap <open_heap>)
    currNode = open_heap.top();      // find the node with the least 'f' on the open list, call it 'q'
    open_heap.pop();                 // pop 'q' off the 'minimum heap'
    open.erase(currNode);            // erase 'q' from the open list
    closed.insert(currNode);         // add 'q' to closed list
    auto time_now = std::chrono::high_resolution_clock::now();

    if ((float)std::chrono::duration_cast<std::chrono::microseconds>(time_now - time_start).count() / 1000 > timeout_threshold_) 
    {
      if (verbose_) {
        printf(ANSI_COLOR_CYAN "[JPS]: Planning timeout! Using current best node as goal.\n" ANSI_COLOR_RESET);
      }
      planning_status_ = PlanningState::PATH_INCOMPLETE;

      break;
    }
    octomap::point3d currCoord = keyToCoord_modified(currNode.key, tree);
    // if the current node is the goal 
    if (distEuclidean(currCoord, goalCoord_) < planning_tree_resolution_) 
    {
      if(verbose_)
      {
        auto time_end = std::chrono::high_resolution_clock::now();
        printf(ANSI_COLOR_CYAN "[JPS]: Goal Reached!" ANSI_COLOR_RESET "\n");
        printf(ANSI_COLOR_GREEN "--> Time in JPS is %f ms" ANSI_COLOR_RESET "\n", (float)std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count() / 1000);
        printf(ANSI_COLOR_GREEN "--> heuristic type: %d (Manhattan = 0, Euclidean=1, Diagonal=2)" ANSI_COLOR_RESET "\n", (int) heuristic_function_type);
      }
      planning_status_ = PlanningState::PATH_COMPLETE;

      break;
    }
    
    std::vector<Node>   Succ_nodeSet;
    std::vector<double> Succ_costSet;
    // ROS_INFO_STREAM(keyToCoord_modified(currNode.key, tree));
    // ROS_INFO("current Node: dx=%d, dy=%d, dz=%d\n", currNode.dx, currNode.dy, currNode.dz);

    //get the succession
    getJpsSucc(currNode, Succ_nodeSet, Succ_costSet, tree);

    // ROS_INFO("Succ_nodeSet size: %d", (int)Succ_nodeSet.size());

    // Process successors
    for (int i = 0; i < (int)Succ_nodeSet.size(); ++i)
    {
      //see if we can improve the value of succstate
      Node childNode        = Succ_nodeSet.at(i);
      double tentative_gval = currNode.g + Succ_costSet.at(i); // update g(n) for new child node

      auto open_query   = open.find(childNode);
      auto closed_query = closed.find(childNode);

      if (open_query == open.end() && closed_query == closed.end()) {
        childNode.g = tentative_gval;
        childNode.f = childNode.g + childNode.h;
        open_heap.push(childNode);
        open.insert(childNode);
        parent_map[childNode] = currNode;

      } // if this child node is new
      else if (open_query != open.end() && closed_query == closed.end()) {

        // If the cumulative cost of the child node is less than the previous value in the child node, we need to update the child node g(n) value and change the child node parent. In this version, we create a parent map to save the success information
        if (tentative_gval < childNode.g)
        {
          childNode.g = tentative_gval;
          childNode.f = childNode.g + childNode.h;
          parent_map[childNode] = currNode;
          // fval = childNode.g + childNode.h
          childNode.dx = childNode.key.k[0] - currNode.key.k[0];
          childNode.dy = childNode.key.k[1] - currNode.key.k[1];
          childNode.dz = childNode.key.k[2] - currNode.key.k[2];
          // normalization
          if(childNode.dx != 0) {
            childNode.dx /= std::abs(childNode.dx);
          }
          if(childNode.dy != 0) {
            childNode.dy /= std::abs(childNode.dy);
          }
          if(childNode.dz != 0) {
            childNode.dz /= std::abs(childNode.dz);
          }
        }
      } // if current child node is in open set, we need to update the value of it
    }

    // TODO show iteration
    // if (verbose_){
    //   printf(ANSI_COLOR_CYAN "[JPS]: expansion iteration [%d]" ANSI_COLOR_RESET "\n", expand_iteration);
    // }

    if(maxExpand > 0 && expand_iteration >= maxExpand) 
    {
      if(verbose_) {
        printf(ANSI_COLOR_CYAN "[JPS]: MaxExpandStep [%d] Reached!  Using current best node as goal." ANSI_COLOR_RESET "\n\n", maxExpand);
      }
      planning_status_ = PlanningState::PATH_INCOMPLETE;

      break;
    }

    if(open_heap.empty()) 
    {
      if(verbose_) {
        printf(ANSI_COLOR_CYAN "[JPS]: Minimum heap is empty!" ANSI_COLOR_RESET "\n\n");
      }
      // TODO
      planning_status_ = PlanningState::ERROR;

      return false;
    }
  }

  // if (currNode.key == startNode.key) {
  //   if (verbose_) {
  //     printf(ANSI_COLOR_RED "[JPS]: start 'Node' is identical with the current 'Node'!\n\n" ANSI_COLOR_RESET);
  //   }
  //   planning_status_ = PlanningState::ERROR;

  //   return false;
  // }

  if(verbose_) {
    printf(ANSI_COLOR_CYAN "[JPS]: goal g: %f, h: %f!" ANSI_COLOR_RESET "\n", currNode.g, currNode.h);
    printf(ANSI_COLOR_CYAN "[JPS]: Expand [%d] nodes!" ANSI_COLOR_RESET "\n", expand_iteration);
  }

  path_Keys_ = backtrackPathKeys(currNode, startNode); // back trace
  
  return true;
}

double GraphSearch::DPF_Plan(octomap::OcTree &tree, std::unordered_set<octomap::OcTreeKey, DPF_HashFunction> &region_set, std::unordered_map<octomap::OcTreeKey, double, DPF_HashFunction> &dist_map, const double &potential_weight)
{
  // clear previous value
  open.clear();
  closed.clear();
  parent_map.clear();
  std::priority_queue<Node, std::vector<Node>, CostComparator> null_heap;
  swap(open_heap, null_heap);
  path_Keys_.clear();

  //octreeKey of start_coord and goal_coord
  octomap::OcTreeKey startKey = tree.coordToKey(startCoord_);
  octomap::OcTreeKey goalKey  = tree.coordToKey(goalCoord_);

  Node startNode(startKey, 0, 0, 0);
  startNode.g   = dist_map[startKey];
  startNode.h   = getHeu(startKey, goalKey, tree);
  startNode.f   = startNode.g + startNode.h;

  // Insert start node
  open_heap.push(startNode);
  open.insert(startNode);

  int expand_iteration = 0;
  Node currNode;
  // this is the main loop
  while (!open.empty() && ros::ok()) 
  {
    expand_iteration++;
    // get element with smallest cost (we initially built a minimum heap)
    currNode = open_heap.top();      // find the node with the least 'f' on the open list, call it 'q'
    open_heap.pop();             // pop 'q' off the 'minimum heap'
    open.erase(currNode);            // erase 'q' from the open list
    closed.insert(currNode);         // add 'q' to closed list

    octomap::point3d currCoord = keyToCoord_modified(currNode.key, tree);
    // if the current node is the goal 
    if (distEuclidean(currCoord, goalCoord_) < planning_tree_resolution_) 
    {
      if(verbose_)
      {
        printf(ANSI_COLOR_CYAN "[DPF Planner]: Goal Reached!" ANSI_COLOR_RESET "\n");
        printf(ANSI_COLOR_GREEN "--> heuristic type: %d (Manhattan = 0, Euclidean=1, Diagonal=2)" ANSI_COLOR_RESET "\n", (int) heuristic_function_type);
      }
      planning_status_ = PlanningState::PATH_COMPLETE;

      break;
    }
    
    std::vector<Node>   Succ_nodeSet;
    std::vector<double> Succ_costSet;

    //get the succession
    getSucc(currNode, Succ_nodeSet, Succ_costSet, tree, region_set, dist_map, potential_weight);

    // Process successors
    for (int i = 0; i < (int) Succ_nodeSet.size(); ++i)
    {
      //see if we can improve the value of succstate
      Node childNode        = Succ_nodeSet.at(i);
      double tentative_gval = currNode.g + Succ_costSet.at(i); // update g(n) for new child node
      
      auto open_query   = open.find(childNode);
      auto closed_query = closed.find(childNode);

      if (open_query == open.end() && closed_query == closed.end()) {
        childNode.g           = tentative_gval;
        childNode.f           = childNode.g + childNode.h;
        open_heap.push(childNode);
        open.insert(childNode);
        parent_map[childNode] = currNode;
      } // childNode is new
      else if (open_query != open.end() && closed_query == closed.end()) {
        // If the cumulative cost of the child node is less than the previous value in the child node, we need to update the child node g(n) value and change the child node parent. In this version, we create a parent map to save the success information
        if (tentative_gval < childNode.g)
        {
          childNode.g           = tentative_gval;
          childNode.f           = childNode.g + childNode.h;
          parent_map[childNode] = currNode;
          // fval = childNode.g + childNode.h
        }
      } // childNode in open set already
    }

    if(open_heap.empty()) 
    {
      if (verbose_) {
        printf(ANSI_COLOR_RED "[DPF Planner]: Minimum heap is empty!" ANSI_COLOR_RESET "\n\n");
      }
      planning_status_ = PlanningState::ERROR;

      return std::numeric_limits<double>::infinity();
    }

  }

  // if (currNode.key == startNode.key) {
  //   if (verbose_) {
  //     printf(ANSI_COLOR_RED "[DPF Planner]: start <Node> is identical with the current <Node>!" ANSI_COLOR_RESET "\n\n");
  //   }
  //   planning_status_ = PlanningState::ERROR;

  //   return std::numeric_limits<double>::infinity();
  // }

  if(verbose_) {
    printf(ANSI_COLOR_CYAN "[DPF Planner]: goal g: %f, h: %f!" ANSI_COLOR_RESET "\n", currNode.g, currNode.h);
    printf(ANSI_COLOR_CYAN "[DPF Planner]: Expand [%d] nodes!" ANSI_COLOR_RESET "\n", expand_iteration);
  }
  path_Keys_ = backtrackPathKeys(currNode, startNode);
  
  return currNode.g;
}

void GraphSearch::getSucc(Node& currSucc, std::vector<Node>& NodeSet, std::vector<double>& Succ_costSet, octomap::OcTree &tree, std::unordered_set<octomap::OcTreeKey, DPF_HashFunction> &region_set, std::unordered_map<octomap::OcTreeKey, double, DPF_HashFunction> &dist_map, const double &potential_weight)
{
  octomap::OcTreeKey goalKey = tree.coordToKey(goalCoord_);

  for (const auto d: EXPANSION_DIRECTIONS) 
  {
    auto new_key    = expand(currSucc.key, d[0], d[1], d[2]);
    auto currPoint  = keyToCoord_modified(new_key, tree);
    auto currNode   = tree.search(new_key);


    if (currNode != NULL) 
    {
      // if current neighbor node is occupied, we need to find the next neighbor
      if (!isFree(currNode, currPoint)) {
        continue;
      }
      // if current neighbor node is not located in the search region, we need to find the next neighbor
      auto region_query = region_set.find(new_key);
      if (region_query == region_set.end()) {
        continue;
      }

      Node newNode;
      newNode.key = new_key;
      newNode.h   = getHeu(new_key, goalKey, tree);

      NodeSet.push_back(newNode);
      Succ_costSet.push_back(distEuclidean(new_key, currSucc.key, tree) + potential_weight*dist_map[new_key]);
    } 
  }
}


void GraphSearch::getJpsSucc(Node& currSucc, std::vector<Node>& NodeSet, std::vector<double>& Succ_costSet, const octomap::OcTree &tree)
{
  const int norm1     = std::abs(currSucc.dx)+std::abs(currSucc.dy)+std::abs(currSucc.dz); // expansion type definition 
  int       num_neib  = jn3d_->nsz[norm1][0];                                              // number of the possible natural neighbors
  int       num_fneib = jn3d_->nsz[norm1][1];                                              // number of the possible forced neighbors
  int       id        = (currSucc.dx+1)+3*(currSucc.dy+1)+9*(currSucc.dz+1);               // 3x3x3 cubic id, indicating the moving directions

  // get goal key
  octomap::OcTreeKey goalKey = tree.coordToKey(goalCoord_);

  for( int dev = 0; dev < num_neib+num_fneib; ++dev) 
  {
    octomap::OcTreeKey newKeySucc;
    int dx, dy, dz;

    // begin from natural neighbors 
    if(dev < num_neib) { 
      dx = jn3d_->ns[id][0][dev];
      dy = jn3d_->ns[id][1][dev];
      dz = jn3d_->ns[id][2][dev];
      // printf(ANSI_COLOR_GREEN "current increment: dx=%d, dy=%d, dz=%d\n" ANSI_COLOR_RESET, dx, dy, dz);
      // keep moving to find the jump point
      if(!jump(currSucc.key, dx, dy, dz, newKeySucc, tree)) {
        continue;
      } // if current direction can not find any jump point, chose the next natural neighbor direction to do search
    }
    else { // begin from the forced neighbors if natural neighbors have been checked

      // 'nextKey' is the check point key value, which indicates the possible obstacles'position, which may lead to the generation of the forced neighbors.
      // [Note]: We use 'switch-case' grammer to describe the mapping between check points and forced neighbors, and a certain check point may mapping more than one forced neighbors. Therefore, the parament in f1[id][x][check] is identical to f2[id][x][forced].
      octomap::OcTreeKey   nextKey   = expand(currSucc.key, jn3d_->f1[id][0][dev-num_neib], jn3d_->f1[id][1][dev-num_neib], jn3d_->f1[id][2][dev-num_neib]); 
      octomap::point3d     currPoint = keyToCoord_modified(nextKey, tree);
      octomap::OcTreeNode* currNode  = tree.search(nextKey);

      if(isOccupied(currNode, currPoint)) { 
        dx = jn3d_->f2[id][0][dev-num_neib];
        dy = jn3d_->f2[id][1][dev-num_neib];
        dz = jn3d_->f2[id][2][dev-num_neib];
        if(!jump(currSucc.key, dx, dy, dz, newKeySucc, tree)) {
          continue;
        }
      } // if find one 'nextKey' position is occupied, there must exists at least one forced neighbor 
      else {
        continue;
      } // if the check point is not occupied, which means the relative position will not exist a forced neighbor.
    }
    // Jump points need to be added to the OpenList, the jump point key value is equal to 'newKeySucc'
    Node newNode;
    newNode.key = newKeySucc;
    newNode.h   = getHeu(newKeySucc, goalKey, tree);
    newNode.dx  = dx;
    newNode.dy  = dy;
    newNode.dz  = dz;

    NodeSet.push_back(newNode);
    Succ_costSet.push_back(distEuclidean(newKeySucc, currSucc.key, tree));

  }

}

double GraphSearch::getHeu(const octomap::OcTreeKey &key1, const octomap::OcTreeKey &key2, const octomap::OcTree &tree)
{
  constexpr float sqrt_3 = 1.7320508;
  constexpr float sqrt_2 = 1.4142136;

  double h = 0.0;
  octomap::point3d point1 = keyToCoord_modified(key1, tree);
  octomap::point3d point2 = keyToCoord_modified(key2, tree);
  double dx = std::fabs(point1.x() - point2.x());
  double dy = std::fabs(point1.y() - point2.y());
  double dz = std::fabs(point1.z() - point2.z());
  if (heuristic_function_type == HeuristicFunctionType::Manhattan) {
      h = dx + dy + dz;
  } 
  else if (heuristic_function_type == HeuristicFunctionType::Euclidean) {
      h = (point1 - point2).norm();
  } 
  else if (heuristic_function_type == HeuristicFunctionType::Diagonal) {
      double min_xyz = std::min({dx, dy, dz});
      double max_xyz = std::max({dx, dy, dz});
      double mid_xyz = dx + dy + dz - min_xyz - max_xyz;
      h = (sqrt_3 - sqrt_2) * min_xyz + (sqrt_2 - 1) * mid_xyz + max_xyz;
  } 

  return h * eps_;
}

bool GraphSearch::jump(octomap::OcTreeKey &currKey, const int &dx, const int &dy, const int &dz, octomap::OcTreeKey &newKey, const octomap::OcTree &tree) 
{
  // follow current direction to find the next point
  newKey = expand(currKey, dx, dy, dz);
  // ROS_INFO_STREAM("debug 1111 " << keyToCoord_modified(newKey, tree));

  octomap::OcTreeNode* newNode  = tree.search(newKey);
  octomap::point3d     newCoord = keyToCoord_modified(newKey, tree);
  // if n is an obstacle or is outside the gird
  if (!isFree(newNode, newCoord)) {
    return false;
  }
  // if n reach the goal
  if (distEuclidean(newCoord, goalCoord_) < planning_tree_resolution_) {
    return true;
  }
  // if n has at least one forced neighbor
  if (hasForced(dx, dy, dz, newKey, tree)) {
    return true;
  }

  const int id       = (dx+1)+3*(dy+1)+9*(dz+1);
  const int norm1    = std::abs(dx) + std::abs(dy) +std::abs(dz);
  int       num_neib = jn3d_->nsz[norm1][0]; // number of natural neighbors, leading to limited expansion directions

  // so we need to 'jump' all directions of current node to find a jump point
  // [Note]: 'num_neib-1' means we consider the directions except current expansion direction. You can check the Neib() function in 'jn3d_', and will find that the maximum 'dev' case is always 'tx = dx; ty = dy; tz = dz;', which can guarantee the moving directions will not contain current direction and will satisfy any 'norm1' situations. What is more, the direction division can always follow the vector division rules.

  // The situation 'norm1 = 0' will never happen in this jump() function. 
  for (int k = 0; k < num_neib - 1; ++k) 
  {
    octomap::OcTreeKey new_newKey;
    if(jump(newKey, jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k], new_newKey, tree)) {
      return true;
    } 
  } 
  // keep moving at current direction
  return jump(newKey, dx, dy, dz, newKey, tree); 
}

bool GraphSearch::hasForced(const int &dx, const int &dy, const int &dz, const octomap::OcTreeKey &key, const octomap::OcTree &tree) 
{
  int id    = (dx+1)+3*(dy+1)+9*(dz+1);                   // locate the 3x3x3 expansion cubic
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz); // expansion type

  // check specifc positions to recognize if there exists forced neighbors around in 3x3x3 cubic
  // If a check point around current node is occupied, that means there exists at least one forced neighbors so this function can return 'true'.
  switch(norm1)
  {
    // 1D movement, check 8 neighbors
    case 1:
      for (int cn = 0; cn < 8; ++cn)
      {
        octomap::OcTreeKey   newKey   = expand(key, jn3d_->f1[id][0][cn], jn3d_->f1[id][1][cn], jn3d_->f1[id][2][cn]);
        octomap::OcTreeNode* newNode  = tree.search(newKey);
        octomap::point3d     newPoint = keyToCoord_modified(newKey, tree);
        if (isOccupied(newNode, newPoint)) {
          return true;
        }
      }
      return false;
    // 2D movement, check 8 neighbors
    case 2:
      for(int cn = 0; cn < 8; ++cn)
      {
        octomap::OcTreeKey   newKey   = expand(key, jn3d_->f1[id][0][cn], jn3d_->f1[id][1][cn], jn3d_->f1[id][2][cn]);
        octomap::OcTreeNode* newNode  = tree.search(newKey);
        octomap::point3d     newPoint = keyToCoord_modified(newKey, tree);
        if( isOccupied(newNode, newPoint) ) {
          return true;
        }
      }
      return false;
    // 3D movement, check 6 neighbors
    case 3:
      for( int cn = 0; cn < 6; ++cn )
      {
        octomap::OcTreeKey   newKey   = expand(key, jn3d_->f1[id][0][cn], jn3d_->f1[id][1][cn], jn3d_->f1[id][2][cn]);
        octomap::OcTreeNode* newNode  = tree.search(newKey);
        octomap::point3d     newPoint = keyToCoord_modified(newKey, tree);
        if( isOccupied(newNode, newPoint) ) {
          return true;
        }
      }
      return false;
    // no movement
    default:
      return false;
  }
}


bool GraphSearch::isOccupied(octomap::OcTreeNode* currNode, octomap::point3d &point) {
  // default occupancy probability threshold is set to 0.7
  // the `value` store the log-odds value, so getOccupancy run probability() function to do type transfer
  return (currNode!=NULL && currNode->getOccupancy() > 0.5) && (point.x() < xDim_ && point.x() > -xDim_ 
                  && point.y() < yDim_ && point.y() > -yDim_ && point.z() < zDim_ && point.z() >= 0);
}

bool GraphSearch::isFree(octomap::OcTreeNode* currNode, octomap::point3d &point) {
  return (currNode!=NULL && currNode->getOccupancy() <= 0.5) 
          && (point.x() < xDim_ && point.x() > -xDim_ && point.y() < yDim_ && point.y() > -yDim_ && point.z() < zDim_ && point.z() >= 0);
}

double GraphSearch::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {
  return (p1 - p2).norm();
}

double GraphSearch::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTree &tree) {
  double voxel_dist = sqrt((k1.k[0] - k2.k[0])*(k1.k[0] - k2.k[0]) + (k1.k[1] - k2.k[1])*(k1.k[1] - k2.k[1]) + (k1.k[2] - k2.k[2])*(k1.k[2] - k2.k[2]));
  return voxel_dist * tree.getResolution();
}

octomap::OcTreeKey GraphSearch::expand(const octomap::OcTreeKey &key, const int &dx, const int &dy, const int &dz) 
{
  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + dx;
  k.k[1] = key.k[1] + dy;
  k.k[2] = key.k[2] + dz;

  return k;
}

octomap::point3d GraphSearch::keyToCoord_modified(const octomap::OcTreeKey &NodeKey, const octomap::OcTree &tree) {
  octomap::point3d point = tree.keyToCoord(NodeKey);
  octomap::point3d point_modified(point.x() - 0.5 * planning_tree_resolution_, point.y() - 0.5 * planning_tree_resolution_, point.z() - 0.5 * planning_tree_resolution_);
  
  return point_modified;
}

std::vector<octomap::OcTreeKey> GraphSearch::backtrackPathKeys(const Node &from, const Node &to) {
  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  keys.push_back(current.key);

  while (current.key != to.key) {
    current = parent_map.find(current)->second; // parent_map: <(key)first Node: Child, (value)second Node: Parent, HashFunction> 
    keys.push_back(current.key);
  };

  keys.push_back(to.key);

  // reverse order
  std::reverse(keys.begin(), keys.end());

  return keys;
}

std::vector<octomap::OcTreeKey> GraphSearch::getPathKeys() const {
  return path_Keys_;
}

std::vector<Node> GraphSearch::getOpenSet() const {
  std::vector<Node> openSet;
  for (auto it = open.begin(); it != open.end(); it++) {
    openSet.push_back(*it);
  }
  
  return openSet;
}

std::vector<Node> GraphSearch::getCloseSet() const {
  std::vector<Node> closeSet;
  for (auto it = closed.begin(); it != closed.end(); it++) {
    closeSet.push_back(*it);
  }
  
  return closeSet;
}

PlanningState GraphSearch::getPlanningStatus() const {return planning_status_;};

void GraphSearch::SetHeuristic(std::string & heu_type)
{
    // Manhattan = 0, Euclidean=1, Diagonal=2
    if (heu_type == "Manhattan") {
        heuristic_function_type = HeuristicFunctionType::Manhattan;
    } else if (heu_type == "Euclidean") {
        heuristic_function_type = HeuristicFunctionType::Euclidean;
    } else if (heu_type == "Diagonal") {
        heuristic_function_type = HeuristicFunctionType::Diagonal;
    } else {
        heuristic_function_type = HeuristicFunctionType::Diagonal;
    }
}

template <typename T>
bool GraphSearch::parse_param(const std::string &param_name, T &param_dest, const ros::NodeHandle &nh) 
{
  if (!nh.getParam(param_name, param_dest)) {
    ROS_ERROR_STREAM("[rokae_navigation_JPS]: Could not load param '" << param_name.c_str() <<"'");

    return false;
  } 
  else {
    ROS_INFO_STREAM("[rokae_navigation_JPS]: Loaded '" << param_name << "' = '" << param_dest << "'");

    return true;
  }
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        // norm1 = 0: no move; norm1 = 1: straight; norm1 = 2: diagonal sqrt(2); norm1 = 3: diagonal sqrt(3)
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz); 
        for (int dev = 0; dev < nsz[norm1][0]; ++dev){
          Neib(dx, dy, dz, norm1, dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        }

        for (int dev = 0; dev < nsz[norm1][1]; ++dev) {
          FNeib(dx,dy,dz,norm1,dev,
              f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
              f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
        }
        // [id] notes the movement direction
        id++;
      }
    }
  }
}


void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev, int &tx, int &ty, int &tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0:  tx=1;  ty=0;  tz=0;  return;
        case 1:  tx=-1; ty=0;  tz=0;  return;
        case 2:  tx=0;  ty=1;  tz=0;  return;
        case 3:  tx=1;  ty=1;  tz=0;  return;
        case 4:  tx=-1; ty=1;  tz=0;  return;
        case 5:  tx=0;  ty=-1; tz=0;  return;
        case 6:  tx=1;  ty=-1; tz=0;  return;
        case 7:  tx=-1; ty=-1; tz=0;  return;
        case 8:  tx=0;  ty=0;  tz=1;  return;
        case 9:  tx=1;  ty=0;  tz=1;  return;
        case 10: tx=-1; ty=0;  tz=1;  return;
        case 11: tx=0;  ty=1;  tz=1;  return;
        case 12: tx=1;  ty=1;  tz=1;  return;
        case 13: tx=-1; ty=1;  tz=1;  return;
        case 14: tx=0;  ty=-1; tz=1;  return;
        case 15: tx=1;  ty=-1; tz=1;  return;
        case 16: tx=-1; ty=-1; tz=1;  return;
        case 17: tx=0;  ty=0;  tz=-1; return;
        case 18: tx=1;  ty=0;  tz=-1; return;
        case 19: tx=-1; ty=0;  tz=-1; return;
        case 20: tx=0;  ty=1;  tz=-1; return;
        case 21: tx=1;  ty=1;  tz=-1; return;
        case 22: tx=-1; ty=1;  tz=-1; return;
        case 23: tx=0;  ty=-1; tz=-1; return;
        case 24: tx=1;  ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}

void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      // moving along One-Axis, the check nodes plane is next to the forced neighbor nodes plane and the direction is the positive direction of the axis moving along.
      nx = fx; ny = fy; nz = dz; // default: moving along the 'z' axis, so check nodes located at the 'xy' plane
      // switch order if different direction
      if(dx != 0){               // moving along the 'x' axis, so check nodes located at the 'yz' plane plane
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){              // moving along the 'y' axis, so check nodes located at the 'zx' plane
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0: 
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}
