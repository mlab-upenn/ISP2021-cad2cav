#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>
namespace fs = std::filesystem;

#include "auto_mapping_ros/graph_builder_bp.hpp"
#include "auto_mapping_ros/utils.h"
#include "graph_partitioner/partitioner.hpp"
#include "graph_partitioner/utils.hpp"

class OmniverseAdapter {
public:
  static const int DEFAULT_NUM_AGENTS = 2;

  OmniverseAdapter() : n_(ros::NodeHandle()), num_agents_(DEFAULT_NUM_AGENTS) {
    stage_sub_ = n_.subscribe<geometry_msgs::PoseArray>(
        "/simulator/stage", 1, &OmniverseAdapter::landmarkPositionCallback,
        this);
    n_.getParam("num_vehicles", num_agents_);
    ROS_WARN_STREAM("No. of vehicles: " << num_agents_);

    map_ = cv::imread(cad_map_filepath, cv::IMREAD_GRAYSCALE);
    ROS_INFO_STREAM("Read in map of size (col*row): " << map_.cols << "x"
                                                      << map_.rows);
  }

  void landmarkPositionCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    landmark_list_.clear();
    for (const auto& landmark_pose : msg->poses) {
      cv::Point2f landmark_position{landmark_pose.position.x,
                                    landmark_pose.position.y};
      landmark_list_.push_back(landmark_position);
    }
    buildGraph();
  }

private:
  const fs::path amr_package_path = ros::package::getPath("auto_mapping_ros");
  const fs::path simulator_package_path =
      ros::package::getPath("f110_simulator");
  const fs::path cad_map_filepath = amr_package_path / "maps/levine_2.dwg.png";
  const std::string csv_filepath  = amr_package_path / "csv/sequence";

  ros::NodeHandle n_;
  ros::Subscriber stage_sub_;

  cv::Mat map_;
  int num_agents_;
  std::vector<cv::Point2f> landmark_list_;

  void buildGraph() {
    amr::GraphBuilderBP graph_builder(map_);
    if (landmark_list_.size() < 3) {
      ROS_INFO("Landmark List Size: %d", landmark_list_.size());
      return;
    } else {
      graph_builder.build_graph(landmark_list_);
    }
    auto graph = graph_builder.get_graph();

    const auto gp_graph = graph_partitioner::fromUserGraph(graph, true);
    graph_partitioner::GraphPartitioner gp(gp_graph);

    // compute graph partition
    auto subgraphs =
        gp.getPartition(num_agents_, graph_partitioner::PartitionType::METIS);

    // visualize subgraphs
    for (unsigned int i = 0; i < subgraphs.size(); ++i) {
      auto& subgraph  = subgraphs.at(i);
      double tsp_cost = subgraph.updateTSPSequence();
      ROS_WARN("Updated TSP path cost for subgraph %d: %lf", i, tsp_cost);
      const auto current_sequence = subgraph.getTSPSequence();

      amr::visualize_sequence_on_graph(map_, graph, current_sequence, true);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_sequence_generator",
            ros::InitOption::AnonymousName);
  OmniverseAdapter adapter;
  ros::spin();
  return 0;
}
