#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "fast-csv-reader/csv.h"
#include "map_service/map_server.hpp"
#include "map_service/revit_info.hpp"
#include "map_service/utils.hpp"

namespace fs = boost::filesystem;

static std::vector<Eigen::RowVector2d> waypoints;

void waypointRegistrationCallback(int event, int x, int y, int flags, void*) {
  // adds current pixel coordinate to list upon user left click
  if (event == cv::EVENT_LBUTTONDOWN) {
    ROS_INFO_STREAM("Waypoint selected: (" << x << ", " << y << ")");
    waypoints.emplace_back(x, y);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "user_click_waypoint_node");

  // create occupancy map
  cad2cav::map_service::RevitInfo revit_info =
      cad2cav::map_service::utils::readRevitStructure("levine_2.csv");
  cad2cav::map_service::MapServer map_server{0.05};
  map_server.buildFromRevitInfo(revit_info);

  // create window
  std::string window_name = "Waypoint Registration - ESC or Q to Exit";
  cv::namedWindow(window_name, cv::WindowFlags::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name, waypointRegistrationCallback);
  ROS_INFO("Displaying floorplan map...");
  ROS_INFO("\tWindows are marked in yellow, doors are marked in green");

  // get the image and flip it to align with grid map coordinates
  cv::Mat map_image = map_server.toCvImage();

  // display window positions
  for (const auto& window : map_server.getWindows()) {
    auto position_world = window.pos_;
    auto position_grid  = map_server.worldToGridCoordinates(position_world);
    cv::circle(map_image, {position_grid.x(), position_grid.y()}, 4,
               cv::Scalar(0, 255, 255));
  }

  // display door positions
  for (const auto& window : map_server.getDoors()) {
    auto position_world = window.pos_;
    auto position_grid  = map_server.worldToGridCoordinates(position_world);
    cv::circle(map_image, {position_grid.x(), position_grid.y()}, 4,
               cv::Scalar(0, 255, 0));
  }

  char k = 0;
  do {
    cv::imshow(window_name, map_image);
    k = cv::waitKey(1);
  } while (!(k == 27 || k == 'q'));
  cv::destroyWindow(window_name);

  const auto map_info = map_server.getMapInfo();

  // transform all points from pixel coordinates to world coordinates
  for (auto& p : waypoints) {
    Eigen::RowVector2d origin{map_info.origin.position.x,
                              map_info.origin.position.y};
    p = origin + p * map_info.resolution;
  }

  // print result
  ROS_INFO_STREAM("Recorded waypoints:");
  for (const auto& point : waypoints) {
    std::cout << "\t[" << point << "]\n";
  }

  // store result
  std::string filename  = "user_waypoints.csv";
  fs::path package_path = ros::package::getPath("map_service");
  fs::path output_path =
      package_path / fs::path("revit_export") / fs::path(filename);

  std::ofstream file(output_path.string());
  file << "X,Y\n";
  for (const auto& point : waypoints) {
    file << point.x() << "," << point.y() << "\n";
  }

  ROS_INFO("Results has been stored in %s", output_path.string().c_str());

  return 0;
}
