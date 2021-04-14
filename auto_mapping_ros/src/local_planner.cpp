#include <auto_mapping_ros/local_planner.h>

namespace amr {

int LocalPlanner::local_planner_counter_ = 0;

LocalPlanner::LocalPlanner() : node_handle_(std::make_shared<ros::NodeHandle>(ros::NodeHandle())),
                               tf_listener_(tf_buffer_),
                               trajp_(*node_handle_),
                               mpc_(*node_handle_),
                               occ_grid_(*node_handle_),
                               global_planner_(node_handle_),
                               coverage_sequence_(),
                               last_updated_pose_(),
                               current_plan_(),
                               local_planning_mode_("mpc") {
    local_planner_id_ = ++local_planner_counter_;

    std::string package_name, csv_relative_filepath, csv_localtraj_filepath;
    node_handle_->getParam("package_name", package_name);
    node_handle_->getParam("csv_filepath", csv_relative_filepath);
    node_handle_->getParam("csv_localtraj", csv_localtraj_filepath);
    csv_relative_filepath = csv_relative_filepath + "_" + std::to_string(local_planner_id_) + ".csv";

    std::string pose_topic, drive_topic, brake_topic, scan_topic, plan_topic, seq_topic;
    node_handle_->getParam("pose_topic", pose_topic);
    pose_topic = pose_topic + "_" + std::to_string(local_planner_id_);
    node_handle_->getParam("drive_topic", drive_topic);
    drive_topic = drive_topic + "_" + std::to_string(local_planner_id_);
    node_handle_->getParam("brake_topic", brake_topic);
    brake_topic = brake_topic + "_" + std::to_string(local_planner_id_);
    node_handle_->getParam("scan_topic", scan_topic);
    scan_topic = scan_topic + "_" + std::to_string(local_planner_id_);
    plan_topic = "current_plan_" + std::to_string(local_planner_id_);
    seq_topic = "coverage_sequence_" + std::to_string(local_planner_id_);

    node_handle_->getParam("base_frame", base_frame_);
    base_frame_ = "racecar" + std::to_string(local_planner_id_) + "/" + base_frame_;
    node_handle_->getParam("map_frame", map_frame_);

    std::string local_planning_mode;
    node_handle_->getParam("local_planner_mode", local_planning_mode);
    local_planning_mode_ = local_planning_mode;
    ROS_WARN_STREAM("Local planner " << local_planner_id_ << " mode: " << local_planning_mode_);

    pose_sub_ = node_handle_->subscribe(pose_topic, 5, &LocalPlanner::pose_callback, this);
    scan_sub_ = node_handle_->subscribe(scan_topic, 5, &LocalPlanner::scan_callback, this);
    drive_pub_ = node_handle_->advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    brake_pub_ = node_handle_->advertise<std_msgs::Bool>(brake_topic, 1);
    plan_pub_ = node_handle_->advertise<visualization_msgs::Marker>(plan_topic, 1);
    cov_seq_pub_ = node_handle_->advertise<visualization_msgs::Marker>(seq_topic, 1);

    node_handle_->getParam("lookahead_distance", lookahead_distance_);

    // Get ROS Map
    auto input_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(0.1));
    while (input_map == nullptr || input_map->data.empty()) {
        ROS_WARN("Waiting for Map Message on topic: %s", "map");
        input_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(1));
    }
    ROS_INFO("Map Received");

    // Update ROS Map Parameters
    resolution_ = input_map->info.resolution;
    double origin_x = input_map->info.origin.position.x;
    double origin_y = input_map->info.origin.position.y;
    int ros_map_width_cells = input_map->info.width;
    int ros_map_height_cells = input_map->info.height;

    node_handle_->getParam("distance_threshold", distance_threshold_);
    node_handle_->getParam("velocity", velocity_);

    // Update Non ROS Map Params
    double non_ros_map_width;
    node_handle_->getParam("non_ros_map_width", non_ros_map_width);
    double non_ros_map_height;
    node_handle_->getParam("non_ros_map_height", non_ros_map_height);
    bool switch_xy;
    node_handle_->getParam("switch_xy", switch_xy);

    std::vector<std::array<int, 2>> coverage_sequence_non_ros_map;
    const auto csv_filepath = ros::package::getPath(package_name) + csv_relative_filepath;
    std::string csv_localtraj_path = ros::package::getPath(package_name) + csv_localtraj_filepath;
    amr::read_sequence_from_csv(&coverage_sequence_non_ros_map, csv_filepath);

    // Translate non ros sequence to ros
    const auto coverage_sequence_ros_map = fmt_star::translate_sequence_to_ros_coords(coverage_sequence_non_ros_map,
                                                                                      non_ros_map_width,
                                                                                      non_ros_map_height,
                                                                                      switch_xy,
                                                                                      ros_map_width_cells * resolution_,
                                                                                      ros_map_height_cells * resolution_,
                                                                                      origin_x,
                                                                                      origin_y);

    coverage_sequence_ = global_planner_.init(coverage_sequence_ros_map, distance_threshold_);

    ROS_WARN("Coverage sequence in RViz coords for car %d", local_planner_id_);
    for (const auto& p : coverage_sequence_) {
        std::cout << "(" << p[0] << ", " << p[1] << ")" << std::endl;
    }

    std::thread global_planning_thread(&GlobalPlanner::start_global_planner, &global_planner_);
    global_planning_thread.detach();
    ROS_INFO("auto_mapping_ros node is now running!");

    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;
    trajp_.ReadTrajectories(csv_localtraj_path);
    trajp_.Trajectory2world(current_pose_);
    ROS_INFO("Read_trajectories");
    // std::thread t(&LocalPlanner::drive_loop, this);
    // t.detach();
}

void LocalPlanner::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    last_updated_pose_[0] = pose_msg->pose.position.x;
    last_updated_pose_[1] = pose_msg->pose.position.y;
    current_pose_ = pose_msg->pose;
    global_planner_.update_current_position(last_updated_pose_);
    if (!first_pose_estimate_) {
        first_pose_estimate_ = true;
    }
    const auto new_plan = global_planner_.get_new_plan();
    if (new_plan) {
        ROS_INFO("Updated Global Plan for Car %i", local_planner_id_);
        current_plan_ = new_plan.value();
    }
    VisualizePlan();
    VisualizeCoverageSeq();
    float current_angle = Transforms::GetCarOrientation(current_pose_);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);

    if (first_scan_estimate_) {
        // Decide on using which controller to pursue the waypoints
        //  Options: MPC (Model Predictive Control) or Pure Pursuit Algorithm
        //
        if (local_planning_mode_ == "mpc")
            run_mpc(current_plan_, last_updated_pose_, current_state);
        else if (local_planning_mode_ == "pure_pursuit")
            run_pure_pursuit(current_plan_, last_updated_pose_);
        else {
            ROS_ERROR("Unidentified local planning mode! Options: mpc/pure_pursuit");
            exit(1);
        }
    }
}

void LocalPlanner::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    if (first_pose_estimate_) {
        if (!first_scan_estimate_) {
            first_scan_estimate_ = true;
        }
        occ_grid_.FillOccGrid(current_pose_, scan_msg);
        occ_grid_.Visualize();
        mpc_.UpdateScan(scan_msg);
    }
}

std::vector<PlannerNode> LocalPlanner::transform(const std::vector<PlannerNode>& reference_way_points,
                                                 const PlannerNode& current_way_point) {
    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link = tf_buffer_.lookupTransform(base_frame_, map_frame_, ros::Time(0));

    std::vector<PlannerNode> transformed_way_points;
    for (const auto& reference_way_point : reference_way_points) {
        geometry_msgs::Pose map_way_point;
        map_way_point.position.x = reference_way_point[0];
        map_way_point.position.y = reference_way_point[1];
        map_way_point.position.z = 0;
        map_way_point.orientation.x = 0;
        map_way_point.orientation.y = 0;
        map_way_point.orientation.z = 0;
        map_way_point.orientation.w = 1;

        tf2::doTransform(map_way_point, map_way_point, map_to_base_link);
        transformed_way_points.emplace_back(PlannerNode{map_way_point.position.x, map_way_point.position.y});
    }
    return transformed_way_points;
}

Input LocalPlanner::GetNextInput() {
    if (inputs_idx_ >= current_inputs_.size()) {
        // ROS_ERROR("Trajectory complete!");
        return Input(0.2, -0.05);
    }
    return current_inputs_[inputs_idx_];
}

int LocalPlanner::get_best_track_point(const std::vector<PlannerNode>& way_point_data) {
    double closest_distance = std::numeric_limits<double>::max();
    PlannerNode best_node{-1, -1};
    int index = -1;
    int best_index = -1;

    for (const auto& way_point : way_point_data) {
        index++;
        if (way_point[0] < 0) continue;
        double distance = sqrt(way_point[0] * way_point[0] + way_point[1] * way_point[1]);
        double lookahead_diff = distance - lookahead_distance_;
        if (lookahead_diff < closest_distance && lookahead_diff > 0) {
            best_index = index;
            closest_distance = lookahead_diff;
            best_node = way_point;
        }
    }

    /*********************************************************************************/
    /* Execute the reverse path is another good option... Save it here for later use */
    if (best_node == PlannerNode{-1, -1}) {
        // Execute Reverse
        index = -1;
        best_index = -1;
        for (const auto& way_point : way_point_data) {
            index++;
            double distance = sqrt(way_point[0] * way_point[0] + way_point[1] * way_point[1]);
            double lookahead_diff = std::abs(distance - lookahead_distance_);
            if (lookahead_diff < closest_distance) {
                best_index = index;
                closest_distance = lookahead_diff;
                best_node = way_point;
            }
        }
        return best_index;
    }
    /*********************************************************************************/

    ROS_DEBUG("closest_way_point: %f, %f", static_cast<double>(best_node[0]), static_cast<double>(best_node[1]));
    return best_index;
}

void LocalPlanner::run_pure_pursuit(const std::vector<PlannerNode>& reference_way_points,
                                    const PlannerNode& current_way_point) {
    const auto transformed_way_points = transform(reference_way_points, current_way_point);
    const auto goal_way_point_index = get_best_track_point(transformed_way_points);

    if (goal_way_point_index != -1) {
        const auto goal_way_point = transformed_way_points[goal_way_point_index];
        const auto steering_angle = 2 * (goal_way_point[1]) / (lookahead_distance_ * lookahead_distance_);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = base_frame_;
        drive_msg.drive.steering_angle =
            steering_angle > 0.4 ? steering_angle : ((steering_angle < -0.4) ? -0.4 : steering_angle);
        ROS_DEBUG("steering angle: %f", steering_angle);
        drive_msg.drive.speed = velocity_;
        drive_pub_.publish(drive_msg);
    } else {
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = base_frame_;
        drive_msg.drive.steering_angle = 0;
        drive_msg.drive.speed = velocity_;
        drive_pub_.publish(drive_msg);
    }
}

void LocalPlanner::run_mpc(const std::vector<PlannerNode>& reference_way_points,
                           const PlannerNode& current_way_point, const State current_state) {
    const auto transformed_way_points = transform(reference_way_points, current_way_point);
    const auto goal_way_point_index = get_best_track_point(transformed_way_points);

    if (goal_way_point_index != -1) {
        const auto goal_way_point = reference_way_points[goal_way_point_index];
        std::pair<float, float> goal_to_track((float)goal_way_point[0], (float)goal_way_point[1]);
        Input input_to_pass = GetNextInput();

        trajp_.Update(current_pose_, occ_grid_, goal_to_track);
        trajp_.Visualize();

        if (trajp_.best_trajectory_index() > -1) {
            vector<State> bestMiniPath = trajp_.best_minipath();
            mpc_.Update(current_state, input_to_pass, bestMiniPath);
            current_inputs_ = mpc_.solved_trajectory();
            mpc_.Visualize();
            inputs_idx_ = 0;
            ackermann_msgs::AckermannDriveStamped drive_msg;
            Input input = GetNextInput();

            if (trajp_.best_trajectory_index() < 0) {
                input.set_v(0.2);
            }
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.drive.speed = input.v();
            drive_msg.drive.steering_angle = input.steer_ang();
            drive_pub_.publish(drive_msg);
            inputs_idx_++;
        }
    }
}

void LocalPlanner::drive_loop() {
    while (true) {
        if (first_pose_estimate_ && first_scan_estimate_) {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            Input input = GetNextInput();
            if (trajp_.best_trajectory_index() < 0) {
                input.set_v(0.2);
            }
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.drive.speed = input.v();
            drive_msg.drive.steering_angle = input.steer_ang();
            drive_pub_.publish(drive_msg);
            int dt_ms = mpc_.dt() * 1000;
            inputs_idx_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        }
    }
}

void LocalPlanner::VisualizePlan() {
    std::vector<pair<float, float>> best_traj;
    for (const auto& node : current_plan_) {
        std::pair<float, float> node_pair(node[0], node[1]);
        best_traj.push_back(node_pair);
    }
    std::vector<geometry_msgs::Point> best_traj_points = Visualizer::GenerateVizPoints(best_traj);
    std::vector<std_msgs::ColorRGBA> best_traj_colors = Visualizer::GenerateVizColors(best_traj, 0, 1, 0);
    plan_pub_.publish(Visualizer::GenerateList(best_traj_points, best_traj_colors));
    // visualizeCmaes();
}

void LocalPlanner::VisualizeCoverageSeq() {
    std::vector<std::pair<float, float>> list_coverage_seq;
    for (const auto& node : coverage_sequence_) {
        std::pair<float, float> node_pair(node[0], node[1]);
        list_coverage_seq.push_back(node_pair);
    }
    auto list_CovSeq_points = Visualizer::GenerateVizPoints(list_coverage_seq);
    auto list_CovSeq_colors = Visualizer::GenerateVizColors(list_coverage_seq, 0, 0, 1);
    cov_seq_pub_.publish(Visualizer::GenerateList(list_CovSeq_points, list_CovSeq_colors,
                                                  visualization_msgs::Marker::LINE_STRIP));
}

}  // namespace amr
