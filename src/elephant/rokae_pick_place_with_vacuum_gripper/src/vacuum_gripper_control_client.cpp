#include <ros/ros.h>
#include <map>
#include "rokae_pick_place_with_vacuum_gripper/GripperState.h"


std::map<std::string, bool> vacuum_gripper_state = {
              { "on", true },
              { "off", false }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vaccum_gripper_control_client");

  ros::NodeHandle nh;
  ros::ServiceClient gripper_client = nh.serviceClient<rokae_pick_place_with_vacuum_gripper::GripperState>("/rokae_arm/vaccum_gripper_control_server");

  rokae_pick_place_with_vacuum_gripper::GripperState srv;

  if (argc > 1 && argc < 3)
  {
    std::string input_arg = (std::string)argv[1];
    auto switch_state = vacuum_gripper_state.find(input_arg);
    if (switch_state != vacuum_gripper_state.end())
    {
      ROS_INFO("Switch vacuum gripper state: %s", input_arg.c_str());
      srv.request.switch_state = vacuum_gripper_state[input_arg];
      ROS_INFO("current request msg: %s", srv.request.switch_state ? "on" : "off");
    }
    else
    {
      ROS_ERROR("Invalid parament parsed to vacuum gripper control server. use 'on' or 'off'.");
      return 1;
    }
  }
  else
  {
    ROS_ERROR("The number of the parament is limited to 1.");
    return 1;
  }

  ros::service::waitForService("/rokae_arm/vaccum_gripper_control_server");
  bool success = gripper_client.call(srv);
  if (success)
  {
    ROS_INFO("Feedback from vacuum_gripper_control server :%s", srv.response.feedback.c_str());
  }
  else 
  {
    ROS_ERROR("Failed to call vacuum_gripper_control service.");
    return 1;
  }
  
  ros::spinOnce();

  return 0;
}


