#include <ros/package.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "auto_mapping_ros/graph_builder_bp.hpp"
#include "auto_mapping_ros/utils.h"
#include "cad2cav_types/utils.hpp"
#include "graph_partitioner/partitioner.hpp"
#include "graph_partitioner/tsp_solver.hpp"

int main(int argc, char** argv) {
  const fs::path amr_package_path = ros::package::getPath("auto_mapping_ros");
  const fs::path simulator_package_path =
      ros::package::getPath("f110_simulator");
  const fs::path cad_map_filepath =
      simulator_package_path / "maps/levine_4.dwg.png";
  const fs::path unreal_waypoints_path =
      amr_package_path / "csv/actorLocation.csv";
  const std::string csv_filepath = (amr_package_path / "csv/sequence").string();

  ros::init(argc, argv, "coverage_sequence_generator",
            ros::InitOption::AnonymousName);
  ros::NodeHandle n;
  int n_agents = 0;
  n.getParam("num_vehicles", n_agents);
  ROS_WARN_STREAM("No. of vehicles: " << n_agents);

  cv::Mat map = cv::imread(cad_map_filepath.string(), cv::IMREAD_GRAYSCALE);
  ROS_INFO_STREAM("Read in map of size (col*row): " << map.cols << "x"
                                                    << map.rows);

  amr::GraphBuilderBP graph_builder(map);
  graph_builder.build_graph(unreal_waypoints_path.string());
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
    amr::write_sequence_to_csv(current_sequence, current_csv_filepath, true);

    amr::visualize_sequence_on_graph(map, graph, current_sequence, true);
  }

  return 0;
}
