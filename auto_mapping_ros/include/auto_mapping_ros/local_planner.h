#ifndef AUTO_MAPPING_ROS_LOCAL_PLANNER_H
#define AUTO_MAPPING_ROS_LOCAL_PLANNER_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <auto_mapping_ros/global_planner.h>
#include <auto_mapping_ros/maneuvers.h>
#include <auto_mapping_ros/utils.h>
#include <fmt_star/planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <auto_mapping_ros/input.hpp>
#include <auto_mapping_ros/mpc.hpp>
#include <auto_mapping_ros/occgrid.hpp>
#include <auto_mapping_ros/trajectory_planner.hpp>
#include <auto_mapping_ros/transforms.hpp>
#include <auto_mapping_ros/visualizer.hpp>
#include <typeinfo>
#include <utility>

// For some weird reasons OSQP must be included after OpenCV. Otherwise there will be type conflicts
#include "OsqpEigen/OsqpEigen.h"

namespace amr {

class LocalPlanner {
public:
    LocalPlanner();

    /// Updates the current pose of the agent
    /// @param pose_msg
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
    int local_planner_id_;
    static int local_planner_counter_;
    bool first_pose_estimate_ = false;
    bool first_scan_estimate_ = false;

    std::shared_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher brake_pub_;
    ros::Publisher plan_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    TrajectoryPlanner trajp_;
    MPC mpc_;
    OccGrid occ_grid_;
    std::vector<Input> current_inputs_;
    std::atomic<unsigned int> inputs_idx_;
    std::pair<float, float> occ_offset_;
    geometry_msgs::Pose current_pose_;

    double distance_threshold_;
    double lookahead_distance_;
    double resolution_;
    double velocity_;

    GlobalPlanner global_planner_;
    std::vector<PlannerNode> coverage_sequence_;
    PlannerNode last_updated_pose_;
    std::vector<PlannerNode> current_plan_;

    std::string base_frame_;
    std::string map_frame_;
    std::string local_planning_mode_;  // "pure_pursuit" or "mpc". Default to "mpc".

    std::vector<PlannerNode> transform(const std::vector<PlannerNode>& reference_way_points,
                                       const PlannerNode& current_way_point);

    Input GetNextInput();
    int get_best_track_point(const std::vector<PlannerNode>& way_point_data);
    PlannerNode get_closest(const std::vector<PlannerNode>& way_point_data);

    void run_pure_pursuit(const std::vector<PlannerNode>& reference_way_points,
                          const PlannerNode& current_way_point);

    void run_mpc(const std::vector<PlannerNode>& reference_way_points,
                 const PlannerNode& current_way_point, const State current_state);

    void drive_loop();
    void VisualizePlan();
};

}  // namespace amr

#endif  //FMT_STAR_LOCAL_PLANNER_H
