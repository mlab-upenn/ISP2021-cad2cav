#include <ros/ros.h>

#include "auto_mapping_ros/local_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "auto_mapping_ros_node");
  ros::NodeHandle nh;

  int n_agents;
  nh.getParam("num_vehicles", n_agents);
  ROS_WARN_STREAM("Initialized scene with " << n_agents << " vehicles");

  std::vector<amr::LocalPlanner> local_planners(n_agents);

  ros::spin();
  return 0;
}
