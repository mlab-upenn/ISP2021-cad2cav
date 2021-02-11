#include "auto_mapping_ros/global_planner.h"

namespace amr {

GlobalPlanner::GlobalPlanner(std::shared_ptr<ros::NodeHandle> node_handle)
    : current_tracking_node_index_(0),
      current_goal_index_(0),
      current_position_{-1, -1},
      node_handle_(std::move(node_handle)),
      client_("fmt_star_server", true),
      first_plan_(true),
      new_plan_available_(false),
      distance_threshold_(0) {
    ROS_INFO("Global Planner is waiting for action server to start.");
    client_.waitForServer();
    ROS_INFO("Action server started");
    std::string planner_name;
    node_handle_->getParam("planner_name", planner_name);
};

std::vector<PlannerNode> GlobalPlanner::init(const std::vector<PlannerNode>& sequence, double distance_threshold) {
    sequence_.clear();
    sequence_ = sequence;
    distance_threshold_ = distance_threshold;
    ROS_INFO("Global Planner Initialized");
    return sequence_;
}

void GlobalPlanner::update_current_position(const PlannerNode& current_position) {
    std::lock_guard<std::mutex> lock(current_position_mutex_);
    current_position_ = current_position;
}

std::optional<std::vector<PlannerNode>> GlobalPlanner::get_new_plan() {
    std::lock_guard<std::mutex> lock(new_plan_mutex_);
    std::vector<PlannerNode> plan{};
    if (new_plan_available_) {
        ROS_INFO("New Plan Available!");
        new_plan_available_ = false;
        plan = new_plan_;
        new_plan_.clear();
        return plan;
    }
    return std::nullopt;
}

void GlobalPlanner::start_global_planner() {
    {
        auto updated_position = PlannerNode{-1, -1};
        while (updated_position == PlannerNode{-1, -1}) {
            ROS_DEBUG("Global Planner waiting for new goal ...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            current_position_mutex_.lock();
            updated_position = current_position_;
            current_position_mutex_.unlock();
        }
    }
    while (ros::ok()) {
        PlannerNode updated_position;
        {
            std::lock_guard<std::mutex> lock(current_position_mutex_);
            updated_position = current_position_;
        }
        const auto current_distance = distance(updated_position, sequence_[current_tracking_node_index_]);
        if (current_distance < distance_threshold_ || first_plan_) {
            ROS_DEBUG("Getting New Plan.");
            if (first_plan_) {
                first_plan_ = false;
            } else {
                current_tracking_node_index_++;
            }
            const auto new_plan = get_next_plan(updated_position);
            {
                std::lock_guard<std::mutex> copy_lock(new_plan_mutex_);
                new_plan_.clear();
                new_plan_ = new_plan;
                new_plan_available_ = true;
            }
            if (current_tracking_node_index_ == sequence_.size() - 1) {
                //TODO: Give Brake Signal
                ROS_INFO("Global Planner Client is now shutting down as the sequence has been explored.");
            }
        }
    }
}

std::vector<PlannerNode> GlobalPlanner::get_next_plan(const PlannerNode& current_position) {
    if (sequence_.empty()) {
        ROS_ERROR("The coverage sequence is empty. It needs to be set before calling get_next_plan.");
    }
    update_start(current_position);
    update_end();
    return find_plan();
}

void GlobalPlanner::update_start(const PlannerNode& current_position) {
    ROS_INFO("Start Position: %f, %f", current_position[0], current_position[1]);
    start_.pose.position.x = current_position[0];
    start_.pose.position.y = current_position[1];
    start_.pose.position.z = 0;
    start_.pose.orientation.w = 0;
    start_.pose.orientation.x = 0;
    start_.pose.orientation.y = 0;
    start_.pose.orientation.z = 1;
}

void GlobalPlanner::update_end() {
    ROS_INFO("End Position: %f, %f", sequence_[current_goal_index_][0], sequence_[current_goal_index_][1]);
    end_.pose.position.x = sequence_[current_goal_index_][0];
    end_.pose.position.y = sequence_[current_goal_index_][1];
    end_.pose.position.z = 0;
    end_.pose.orientation.w = 0;
    end_.pose.orientation.x = 0;
    end_.pose.orientation.y = 0;
    end_.pose.orientation.z = 1;
    current_goal_index_++;
}

std::vector<PlannerNode> GlobalPlanner::find_plan() {
    fmt_star::planGoal goal;
    goal.start_position = start_;
    goal.end_position = end_;
    goal.update_map = false;
    goal.update_samples = true;

    client_.sendGoal(goal);
    bool finished_before_timeout = client_.waitForResult(ros::Duration(5.0));

    std::vector<PlannerNode> current_plan{};
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = client_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            auto path = client_.getResult()->path.poses;
            for (const auto& node : path) {
                std::array<double, 2> global_path_node{};
                global_path_node[0] = node.pose.position.x;
                global_path_node[1] = node.pose.position.y;
                current_plan.emplace_back(global_path_node);
            }
        }
    } else {
        ROS_INFO("No Plan Received. Action did not finish before the time out.");
    }
    return current_plan;
}

}  // namespace amr
