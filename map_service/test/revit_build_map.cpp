#include <ros/ros.h>

#include "map_service/map_server.hpp"
#include "map_service/revit_info.hpp"
#include "map_service/utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_revit_publisher_test");
  cad2cav::map_service::RevitInfo revit_info =
      cad2cav::map_service::utils::readRevitStructure("levine_2.csv");

  cad2cav::map_service::MapServer map_server{0.05};
  map_server.buildFromRevitInfo(revit_info);

  ros::Rate loop_rate{1};

  while (ros::ok()) {
    if (map_server.map_pub.getNumSubscribers() > 0) {
      map_server.publish();
      ROS_INFO("Map published.");
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
