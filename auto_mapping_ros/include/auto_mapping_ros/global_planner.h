#ifndef AUTO_MAPPING_ROS_GLOBAL_PLANNER_H
#define AUTO_MAPPING_ROS_GLOBAL_PLANNER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "auto_mapping_ros/utils.h"
#include "fmt_star/planAction.h"

namespace amr {

using PlannerNode = std::array<double, 2>;
typedef actionlib::SimpleActionClient<fmt_star::planAction> Client;

class GlobalPlanner {
public:
  /// Constructs GlobalPlanner (Does not initialize the sequence)
  /// @param node_handle
  explicit GlobalPlanner(std::shared_ptr<ros::NodeHandle> node_handle);

  /// Initializes the Global Planner with the sequence (coordinates of sequence
  /// already same as in ROS Map)
  /// @param sequence
  std::vector<PlannerNode> init(const std::vector<PlannerNode> &sequence,
                                double distance_threshold);

  void update_current_position(const PlannerNode &current_position);
  std::optional<std::vector<PlannerNode>> get_new_plan();
  void start_global_planner();

private:
  size_t current_tracking_node_index_;
  size_t current_goal_index_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped end_;
  PlannerNode current_position_;
  std::vector<PlannerNode> sequence_;
  std::vector<PlannerNode> new_plan_;

  std::shared_ptr<ros::NodeHandle> node_handle_;
  Client client_;

  bool first_plan_;
  bool new_plan_available_;
  double distance_threshold_;

  std::mutex current_position_mutex_;
  std::mutex new_plan_mutex_;

  /// Find Path between current position and the next position in the sequence
  /// @param current_position - current position (x, y) in the map
  /// @return
  std::vector<PlannerNode> get_next_plan(const PlannerNode &current_position);

  /// Update the start position as the current position
  /// @param current_position
  void update_start(const PlannerNode &current_position);

  /**
   * @brief Update the start position as the previous end position
   *
   */
  void update_start();

  /// Update the end position to the next index in the sequence
  void update_end();

  /// Finds and returns the plan between start and end by calling the planner
  /// service
  std::vector<PlannerNode> find_plan();
};

}  // namespace amr

#endif  // FMT_STAR_GLOBAL_PLANNER_H
