#include <ros/ros.h>
#include <math.h>
#include <array>
#include <iostream>
#include <fstream>
#include <thread>
#include <toppra/toppra.hpp>
#include <toppra/constraint.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/parametrizer.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/algorithm.hpp>
#include <toppra/solver/seidel.hpp>
#include "rokae_arm_toppra/ToppRa_srv.h"
#include "matplotlibcpp.h"
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <errno.h>
#include <boost/filesystem.hpp>

//! Set red font in printf function
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
//! Set green font in printf function
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
//! Set yellow font in printf function
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif

//! Set magenta font in printf function
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif

//! Reset font color in printf function
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

namespace plt = matplotlibcpp;

void formatVecToMat(const std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>& vec, Eigen::MatrixXd& mat) 
{
  mat.resize(vec.at(0).rows(), vec.size());
  for (size_t i = 0; i < vec.size(); i++) 
  {
    mat.col(i) = vec.at(i);
  }
}

toppra::BoundaryCond makeBoundaryCond(const int order, const std::vector<toppra::value_type> &values)
{
  toppra::BoundaryCond cond;
  cond.order = order;
  cond.values.resize(values.size());
  for (std::size_t i = 0; i < values.size(); i++) cond.values(i) = values[i];
  return cond;
}

void plotdata(toppra::Vectors &pos_data, toppra::Vectors &vel_data, toppra::Vectors &acc_data, toppra::Vector &time,
                                       const int nDof, int show_time, std::string &time_stamp, std::string &dir_path, bool show = false)
{
  size_t data_size = time.size();
  std::string output_path = dir_path + "/trajectory_info_" + time_stamp + ".png";
  std::vector<std::vector<double>> pos_y, vel_y, acc_y; // store every joint data
  std::vector<double> t_x;

  for (size_t i = 0; i < nDof; i++)
  {
    std::vector<double> joint_pos, joint_vel, joint_acc;
    for (size_t k = 0; k < data_size; k++)
    {
      joint_pos.push_back(pos_data[k](i));
      joint_vel.push_back(vel_data[k](i));
      joint_acc.push_back(acc_data[k](i));
    }
    pos_y.push_back(joint_pos);
    vel_y.push_back(joint_vel);
    acc_y.push_back(joint_acc);
  }

  for (size_t k = 0; k < data_size; k++)
  {
    t_x.push_back(time(k));
  }

  plt::figure(1);
  plt::ion();
  plt::suptitle("JOINT Trajectory Data");
  plt::subplot(3, 1, 1);
  plt::ylabel("Position (rad)");
  for (size_t i = 0; i < nDof; i++) {
    // std::string joint_name = "joint" + std::to_string(i+1);
    // plt::named_plot(joint_name, t_x, pos_y.at(i));
    plt::plot(t_x, pos_y.at(i));
  }
  plt::subplot(3, 1, 2);
  plt::ylabel("Velocity (rad/s)");
  for (size_t i = 0; i < nDof; i++) {
    // std::string joint_name = "joint" + std::to_string(i+1);
    // plt::named_plot(joint_name, t_x, vel_y.at(i));
    plt::plot(t_x, vel_y.at(i));
  }
  plt::subplot(3, 1, 3);
  plt::ylabel("Acceleration (rad/s2)");
  for (size_t i = 0; i < nDof; i++) {
    // std::string joint_name = "joint" + std::to_string(i+1);
    // plt::named_plot(joint_name, t_x, acc_y.at(i));
    plt::plot(t_x, acc_y.at(i));
  }
  plt::xlabel("Time (s)");

  plt::save(output_path);
  if (show)
  {
    plt::pause(show_time);
  }
  plt::close();
}

void inspect(toppra::Vector &vec, toppra::Matrix &controllable_sets, toppra::Matrix &feasible_sets, 
                                                      int show_time, std::string &time_stamp, std::string &dir_path, bool show = false)
{
  std::string output_path = dir_path + "/Path-Position_Path-Velocity_" + time_stamp +".png";
  std::vector<double> vec_, controllable_sets_first_, controllable_sets_second_, feasible_sets_first_, feasible_sets_second_, t_;

  if (vec.size() != 0) {
    for (size_t i = 0; i < vec.size(); i++) {
      vec_.push_back(vec(i));
    }
  }

  if (controllable_sets.size() != 0) {
    for (size_t i = 0; i < controllable_sets.size()/2; i++) {
      controllable_sets_first_.push_back(controllable_sets(i, 0));
      controllable_sets_second_.push_back(controllable_sets(i, 1));
    }
  }

  if (feasible_sets.size() != 0) {
    for (size_t i = 0; i < feasible_sets.size()/2; i++) {
      feasible_sets_first_.push_back(feasible_sets(i, 0));
      feasible_sets_second_.push_back(feasible_sets(i, 1));
    }
  }
  
  toppra::Vector t = toppra::Vector::LinSpaced(vec.size(), 0, vec.size()-1);

  for (size_t i = 0; i < t.size(); i++)
  {
    t_.push_back(t(i));
  }

  plt::figure(2);
  plt::named_plot("Feasible sets", t_, feasible_sets_first_, "g-");
  plt::plot(t_, feasible_sets_second_, "g-");
  plt::named_plot("Controllable sets", t_, controllable_sets_first_, "r--");
  plt::plot(t_, controllable_sets_second_, "r--");
  plt::named_plot("Velocity profile", t_, vec_, "b-");
  plt::title("Path-Position Path-velocity plot");
  plt::xlabel("Path position");
  plt::ylabel("Path velocity square");
  plt::legend();

  plt::save(output_path);
  if (show) {
    plt::pause(show_time);
  }
  plt::close();

}


bool toppraCallback(rokae_arm_toppra::ToppRa_srv::Request &req, rokae_arm_toppra::ToppRa_srv::Response &res)
{
  plt::backend("agg");
  const bool  printInfo      = false;
  // const bool  printInfo      = true;
  const int   nDof           = 6;
  const int   joint_set_size = req.joint_configs_on_way.size() / nDof;
  const float start_t        = 0.0;
  const float end_t          = 8.0;
  int         counter        = 0;
  int         show_time      = 1;

  //#### create linear joint-space constraints ####
  toppra::Vector lowerVlimit = Eigen::VectorXd::Zero(nDof);
  toppra::Vector upperVlimit = Eigen::VectorXd::Zero(nDof);
  toppra::Vector lowerAlimit = Eigen::VectorXd::Zero(nDof);
  toppra::Vector upperAlimit = Eigen::VectorXd::Zero(nDof);
  lowerVlimit << -355/180*M_PI, -355/180*M_PI, -355/180*M_PI, -480/180*M_PI, -450/180*M_PI, -705/180*M_PI;
  upperVlimit << 355/180*M_PI, 355/180*M_PI, 355/180*M_PI, 480/180*M_PI, 450/180*M_PI, 705/180*M_PI;
  lowerAlimit << -3.0, -3.0, -3.0, -6.0, -6.0, -6.0;
  upperAlimit << 3.0, 3.0, 3.0, 6.0, 6.0, 6.0;

  toppra::LinearConstraintPtr linear_joint_vel, linear_joint_acc;
  // Create acceleration bounds, then acceleration constraint object
  linear_joint_vel = std::make_shared<toppra::constraint::LinearJointVelocity>(lowerVlimit, upperVlimit);
  linear_joint_acc = std::make_shared<toppra::constraint::LinearJointAcceleration>(lowerAlimit, upperAlimit);
  linear_joint_acc->discretizationType(toppra::DiscretizationType::Interpolation);
  toppra::LinearConstraintPtrs constraints{linear_joint_vel, linear_joint_acc};
  

  // path has the equation: 0 * x ^ 3 + 1 * x ^ 2 + 2 x ^ 1 + 3
  toppra::Vectors positions;
  toppra::Vector  times;

  times = toppra::Vector::LinSpaced(joint_set_size, start_t, end_t);

  toppra::Vector joint_configs_group(nDof);
  for(auto &jc : req.joint_configs_on_way)
  {
    if (counter < nDof) {
      joint_configs_group(counter) = static_cast<toppra::value_type>(jc);
      counter++;
      if (counter == nDof) {
        positions.push_back(joint_configs_group);
        joint_configs_group.setZero();
        counter = 0;
      }
      continue;
    }
  }

  // CubicSpline, NaturalSpline
  toppra::BoundaryCond bc = makeBoundaryCond(2, {0, 0, 0, 0, 0, 0});
  std::array<toppra::BoundaryCond, 2> bc_type {bc, bc};
  toppra::PiecewisePolyPath Spline = toppra::PiecewisePolyPath(positions, times, bc_type);
  toppra::GeometricPathPtr path;
  path = std::make_shared<toppra::PiecewisePolyPath>(Spline);

  //#### create toppra ####
  toppra::PathParametrizationAlgorithmPtr algo = std::make_shared<toppra::algorithm::TOPPRA>(constraints, path);
  toppra::ReturnCode                      rc1  = algo->computePathParametrization(0, 0);
  toppra::ParametrizationData             pd   = algo->getParameterizationData();

  //#### create constant acceleration parametrizer ####
  toppra::Vector gridpoints   = pd.gridpoints;        // Grid-points used for solving the discredited problem.
  toppra::Vector vsquared     = pd.parametrization;   // Output parametrization (squared path velocity)

  std::shared_ptr<toppra::parametrizer::ConstAccel> ca = std::make_shared<toppra::parametrizer::ConstAccel>(path, gridpoints, vsquared);
  toppra::Bound  path_interval = ca->pathInterval();
  // const int      length2       = 10*(path_interval(1) - path_interval(0));
  const int      length2       = 20;

  toppra::Vector times2        = toppra::Vector::LinSpaced(length2, path_interval(0), path_interval(1));

  toppra::Vectors path_pos2;
  path_pos2 = ca->eval(times2, 0);
  toppra::Vectors path_vel2;
  path_vel2 = ca->eval(times2, 1);
  toppra::Vectors path_acc2;
  path_acc2 = ca->eval(times2, 2);

  Eigen::MatrixXd path_pos2_ = Eigen::MatrixXd::Zero(nDof, length2);
  Eigen::MatrixXd path_vel2_ = Eigen::MatrixXd::Zero(nDof, length2);
  Eigen::MatrixXd path_acc2_ = Eigen::MatrixXd::Zero(nDof, length2);

  formatVecToMat(path_pos2, path_pos2_);
  formatVecToMat(path_vel2, path_vel2_);
  formatVecToMat(path_acc2, path_acc2_);

  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "rc1 = " << int(rc1) << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "pd.gridpoints \n " << pd.gridpoints.transpose() << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "pd.parametrization \n " << pd.parametrization.transpose() << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "pd.controllable_sets \n " << pd.controllable_sets.transpose() << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "pd.feasible_sets \n " << pd.feasible_sets.transpose() << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "pd.ret_code = " << int(pd.ret_code) << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "ca->validate() = " << ca->validate() << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "path_interval = " << path_interval << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "path_pos2_\n " << path_pos2_ << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "path_vel2_\n " << path_vel2_ << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "path_acc2_\n " << path_acc2_ << ANSI_COLOR_RESET << std::endl;
  if (printInfo) std::cout << ANSI_COLOR_MAGENTA << "times2 \n " << times.transpose() << ANSI_COLOR_RESET << std::endl;




  // time stamp
  auto now = std::chrono::system_clock::now();
  auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream datetime;
  datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");

  std::string UTC_string = std::to_string(UTC);

  // mkdir
  std::string dir_path = "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/share/" + UTC_string;
  if (!boost::filesystem::is_directory(dir_path))
  {
    printf(ANSI_COLOR_MAGENTA "begin create path: %s" ANSI_COLOR_RESET "\n",dir_path.c_str());
    if (!boost::filesystem::create_directory(dir_path))
    {
      printf(ANSI_COLOR_RED "create_directories failed: %s" ANSI_COLOR_RESET "\n",dir_path.c_str());
      return true;
    }
  } else {
    printf(ANSI_COLOR_RED "%s already exist" ANSI_COLOR_RESET "\n", dir_path .c_str());
  }

  plotdata(path_pos2, path_vel2, path_acc2, times2, nDof, show_time, UTC_string, dir_path,false);
  inspect(pd.parametrization, pd.controllable_sets, pd.feasible_sets, show_time, UTC_string, dir_path,false);

  // pos saver
  std::ofstream outfile_pos;
  std::string output_pos_path = dir_path + "/toppra_joints_pos_" + UTC_string + ".txt";
  outfile_pos.open (output_pos_path, std::ios::out | std::ios::binary);
  
  if (outfile_pos.is_open())
  {
    outfile_pos.flush();
    printf(ANSI_COLOR_MAGENTA "[rokae_toppra_server]: Joint Position intformation recorder has been created." ANSI_COLOR_RESET "\n");
    for (auto &pos : path_pos2)
    {
      // outfile_pos << "JOINT_CONFIGS:" << std::endl;
      for (size_t i = 0; i < pos.size(); i++)
      {
        res.pos.push_back(pos(i));
        outfile_pos << pos(i) << " ";
      }
      outfile_pos << std::endl;
    }
    outfile_pos.close();
  } else {
    printf(ANSI_COLOR_YELLOW "[position]: trajectory output file can not be opened." ANSI_COLOR_RESET "\n");
  }

  // vel saver
  std::ofstream outfile_vel;
  std::string output_vel_path = dir_path + "/toppra_joints_vel_" + UTC_string + ".txt";
  outfile_vel.open (output_vel_path, std::ios::out | std::ios::binary);

  if (outfile_vel.is_open())
  {
    outfile_vel.flush();
    printf(ANSI_COLOR_MAGENTA "[rokae_toppra_server]: Joint Velocity intformation recorder has been created." ANSI_COLOR_RESET "\n");
    for (auto &vel : path_vel2)
    {
      // outfile_vel << "JOINT_VELOCITY:" << std::endl;
      for (size_t i = 0; i < vel.size(); i++)
      {
        res.vel.push_back(vel(i));
        outfile_vel << vel(i) << " ";
      }
      outfile_vel << std::endl;
    }
    outfile_vel.close();
  } else {
    printf(ANSI_COLOR_YELLOW "[velocity]: trajectory output file can not be opened." ANSI_COLOR_RESET "\n");
  }

  // acc saver
  std::ofstream outfile_acc;
  std::string output_acc_path = dir_path + "/toppra_joints_acc_" + UTC_string + ".txt";
  outfile_acc.open (output_acc_path, std::ios::out | std::ios::binary);

  if (outfile_acc.is_open())
  {
    outfile_acc.flush();
    printf(ANSI_COLOR_MAGENTA "[rokae_toppra_server]: Joint Acceleration intformation recorder has been created." ANSI_COLOR_RESET "\n");
    for (auto &acc : path_acc2)
    {
      // outfile_acc << "JOINT_ACCELERATION:" << std::endl;
      for (size_t i = 0; i < acc.size(); i++)
      {
        res.acc.push_back(acc(i));
        outfile_acc << acc(i) << " ";
      }
      outfile_acc << std::endl;
    }
    outfile_acc.close();
  } else {
    printf(ANSI_COLOR_YELLOW "[acceleration]: trajectory output file can not be opened." ANSI_COLOR_RESET "\n");
  }

  // t saver
  std::ofstream outfile_t;
  std::string output_t_path = dir_path + "/toppra_joints_t_" + UTC_string + ".txt";
  outfile_t.open (output_t_path, std::ios::out | std::ios::binary);

  if (outfile_t.is_open())
  {
    outfile_t.flush();
    printf(ANSI_COLOR_MAGENTA "[rokae_toppra_server]: Joint Times intformation recorder has been created." ANSI_COLOR_RESET "\n" );
    // outfile_t << "JOINT_TIME:" << std::endl;
    for (size_t i = 0; i < times2.size(); i++)
    {
      res.t.push_back(times2(i));
      outfile_t << times2(i) << " ";
    }
    outfile_t << std::endl;
    outfile_t.close();
  } else {
    printf(ANSI_COLOR_YELLOW "[time]: trajectory output file can not be opened." ANSI_COLOR_RESET "\n");
  }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rokae_toppra_server");

  ros::AsyncSpinner spinner(0); 
  spinner.start();

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/rokae_arm/toppra", toppraCallback);
  ROS_INFO("Ready to convert trajectory.");
  
  ros::waitForShutdown();

  return 0;
}

  // plt::title("os_cfar_detected_target_point_num");
  // std::vector<float> plot_x, plot_y;
  // std::map<std::string, std::string> param_dict;
  // plot_x.push_back(1.0);
  // plot_y.push_back(1.0);
  // param_dict["marker"] = "*";
  // param_dict["c"] = "red";
  // plt::scatter<float, float>(plot_x, plot_y, 10.0, param_dict);
  // plt::xlabel("x_zhou");
  // plt::ylabel("y_zhou");
  // plt::show();