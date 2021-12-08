#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "auto_mapping_ros/graph_builder_bp.hpp"
#include "cad2cav_types/utils.hpp"
#include "fast_csv_parser/csv.h"
#include "graph_partitioner/partitioner.hpp"
#include "graph_partitioner/tsp_solver.hpp"
#include "map_service/utils.hpp"

namespace fs = boost::filesystem;

class MapServerAdapter {
public:
  const std::string MAP_TOPIC = "/map";

  cv::Mat toCvImage() const {
    return cad2cav::map_service::utils::occupancyGridToCvImage(map_);
  }
  bool isMapReceived() const { return is_map_received_; }

  MapServerAdapter(ros::NodeHandle& n) : n_(n), is_map_received_(false) {
    map_sub_ = n.subscribe<nav_msgs::OccupancyGrid>(
        MAP_TOPIC, 1, &MapServerAdapter::onMapCallback, this);
  }

  void onMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_             = *msg;
    is_map_received_ = true;
  }

  std::vector<Eigen::Vector2i> toPixelCoordinates(
      const std::vector<Eigen::Vector2d>& world_waypoints) const {
    std::vector<Eigen::Vector2i> pixel_waypoints;

    if (!is_map_received_) {
      ROS_WARN("Map is not yet received. Return empty points.");
      return pixel_waypoints;
    }

    Eigen::Vector2d origin{map_.info.origin.position.x,
                           map_.info.origin.position.y};
    double resolution = map_.info.resolution;

    for (const auto& p_world : world_waypoints) {
      Eigen::Vector2i p_pixel =
          Eigen::round(((p_world - origin) / resolution).array()).cast<int>();
      pixel_waypoints.emplace_back(p_pixel);
    }
    return pixel_waypoints;
  }

  std::vector<Eigen::Vector2d> toWorldCoordinates(
      const std::vector<Eigen::Vector2i>& pixel_waypoints) {
    std::vector<Eigen::Vector2d> world_waypoints;

    if (!is_map_received_) {
      ROS_WARN("Map is not yet received. Return empty points.");
      return world_waypoints;
    }

    Eigen::Vector2d origin{map_.info.origin.position.x,
                           map_.info.origin.position.y};
    double resolution = map_.info.resolution;

    for (const auto& p_pixel : pixel_waypoints) {
      Eigen::Vector2d p_world = origin + resolution * p_pixel.cast<double>();
      world_waypoints.emplace_back(p_world);
    }
    return world_waypoints;
  }

  std::vector<std::array<double, 2>> toWorldCoordinates(
      const std::vector<std::array<double, 2>>& pixel_waypoints) {
    std::vector<std::array<double, 2>> world_waypoints;

    if (!is_map_received_) {
      ROS_WARN("Map is not yet received. Return empty points.");
      return world_waypoints;
    }

    Eigen::Vector2d origin{map_.info.origin.position.x,
                           map_.info.origin.position.y};
    double resolution = map_.info.resolution;

    for (const auto& pixel : pixel_waypoints) {
      Eigen::Vector2i p_pixel{pixel[0], pixel[1]};
      Eigen::Vector2d p_world = origin + resolution * p_pixel.cast<double>();
      world_waypoints.push_back({p_world.x(), p_world.y()});
    }
    return world_waypoints;
  }

private:
  ros::Subscriber map_sub_;
  ros::NodeHandle n_;

  nav_msgs::OccupancyGrid map_;
  bool is_map_received_;
};

std::vector<Eigen::Vector2d> readUserWaypoints(
    const std::string user_waypoints_filepath) {
  std::vector<Eigen::Vector2d> user_waypoints;
  try {
    io::CSVReader<2> waypoint_reader(user_waypoints_filepath);
    waypoint_reader.read_header(io::ignore_extra_column, "X", "Y");
    float x = 0.f, y = 0.f;
    while (waypoint_reader.read_row(x, y)) {
      // simply discard z position & store into array
      user_waypoints.emplace_back(x, y);
    }

  } catch (const io::error::can_not_open_file& e) {
    ROS_FATAL("%s", e.what());
    throw e;
  }
  return user_waypoints;
}

int main(int argc, char** argv) {
  const fs::path amr_package_path = ros::package::getPath("auto_mapping_ros");
  const fs::path map_service_package_path =
      ros::package::getPath("map_service");
  const fs::path user_waypoints_filepath =
      map_service_package_path / "revit_export/user_waypoints.csv";
  const std::string csv_filepath = (amr_package_path / "csv/sequence").string();

  ros::init(argc, argv, "coverage_sequence_map_service");
  ros::NodeHandle n;
  int n_agents = 2;
  n.getParam("num_vehicles", n_agents);
  ROS_WARN_STREAM("No. of vehicles: " << n_agents);

  auto waypoints = readUserWaypoints(user_waypoints_filepath.string());
  ROS_INFO_STREAM("Read user-clicked waypoints: success");

  MapServerAdapter adapter(n);

  ROS_INFO("Waiting to receive occupancy map...");
  while (ros::ok() && !adapter.isMapReceived()) {
    ros::spinOnce();
  }
  ROS_INFO("Occupancy map received!");

  // convert world coordinates to pixel coordinates
  cv::Mat map = adapter.toCvImage();
  amr::GraphBuilderBP graph_builder(map.clone());
  graph_builder.build_graph(adapter.toPixelCoordinates(waypoints));
  auto graph = graph_builder.get_graph();

  const auto gp_graph = cad2cav::fromUserGraph(graph, true);
  cad2cav::graph_partitioner::GraphPartitioner gp(gp_graph);

  // compute graph partition
  auto subgraphs = gp.getPartition(
      n_agents, cad2cav::graph_partitioner::PartitionType::METIS);

  // visualize subgraphs
  for (unsigned int i = 0; i < subgraphs.size(); ++i) {
    auto& subgraph = subgraphs.at(i);
    cad2cav::graph_partitioner::TSPSolver solver(subgraph);
    double tsp_cost = solver.updateTSPSequence();
    ROS_WARN("Updated TSP path cost for subgraph %d: %lf", i, tsp_cost);
    const auto current_sequence = solver.getTSPSequence();

    const auto current_csv_filepath =
        csv_filepath + "_" + std::to_string(i + 1) + ".csv";
    amr::write_sequence_to_csv(adapter.toWorldCoordinates(current_sequence),
                               current_csv_filepath, false);

    amr::visualize_sequence_on_graph(map.clone(), graph, current_sequence, true,
                                     false);
  }

  return 0;
}
