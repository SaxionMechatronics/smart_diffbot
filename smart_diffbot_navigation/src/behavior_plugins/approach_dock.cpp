#include "smart_diffbot_navigation/behavior_plugins/approach_dock.hpp"

using namespace std::chrono_literals;

namespace smart_diffbot_navigation
{

ApproachDock::ApproachDock()
: TimedBehavior<ApproachDockAction>()
{
}

ApproachDock::~ApproachDock() = default;

void ApproachDock::marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  markers_ = msg;
  marker_time_ = steady_clock_.now().seconds();
}

void ApproachDock::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Subscribe to marker pose 
  marker_sub_ = node->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "aruco_markers", 10, std::bind(&ApproachDock::marker_callback, this, _1));

  // Get parameters
  nav2_util::declare_parameter_if_not_declared(node, "approach_dock.linear_gain", rclcpp::ParameterValue(1.0));
  node->get_parameter("approach_dock.linear_gain", linear_gain_);
  nav2_util::declare_parameter_if_not_declared(node, "approach_dock.angular_gain", rclcpp::ParameterValue(1.0));
  node->get_parameter("approach_dock.angular_gain", angular_gain_);
  nav2_util::declare_parameter_if_not_declared(node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

}

Status ApproachDock::onRun(const std::shared_ptr<const ApproachDockAction::Goal> command)
{
  // Extract command
  speed_ = command->speed;
  marker_id_ = command->marker_id;
  distance_to_dock_ = command->distance;

  return Status::SUCCEEDED;
}

Status ApproachDock::onCycleUpdate()
{
  // Check if marker was seen
  if(marker_time_ == 0.0){
    RCLCPP_WARN(logger_, "No marker spotted, stopping approach_dock action");
    return Status::FAILED;
  }

  // Check if it is the right marker
  if(markers_->marker_ids[0] != marker_id_){
    RCLCPP_WARN(logger_, "Detected wrong marker, stopping approach_dock action. Expected %d, found %ld.", marker_id_, markers_->marker_ids[0]);
    return Status::FAILED;
  }

  // Check if the marker pose is up to date enough (<1 sec)
  if (steady_clock_.now().seconds() - marker_time_ > 1.0){
    RCLCPP_WARN(logger_, "Lost sight of marker, stopping approach_dock action. Last seen %d seconds ago.", int(steady_clock_.now().seconds() - marker_time_));
    stopRobot();
    return Status::FAILED;
  }

  // Rotate marker to z-up frame
  geometry_msgs::msg::Pose marker_pose;
  marker_pose.position.x = markers_->poses[0].position.z;
  marker_pose.position.y = -markers_->poses[0].position.x;
  marker_pose.position.z = markers_->poses[0].position.y;
  marker_pose.orientation.x = markers_->poses[0].orientation.z;
  marker_pose.orientation.y = markers_->poses[0].orientation.x;
  marker_pose.orientation.z = markers_->poses[0].orientation.y;
  marker_pose.orientation.w = markers_->poses[0].orientation.w;

  // If desired distance to dock is reached, succeed
  if(marker_pose.position.x <= distance_to_dock_){
    stopRobot();
    return Status::SUCCEEDED;
  }

  // If collision is expected, fail
  if (!isCollisionFree()) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting dock approach");
    return Status::FAILED;
  }

  // Get angle between robot and marker point (alpha)
  double alpha = atan2(marker_pose.position.y, marker_pose.position.x);

  // Get angle between robot and marker plane (beta)
  double yaw = tf2::getYaw(marker_pose.orientation);
  double beta = yaw - M_PI;
  if (fabs(beta) > M_PI){beta -= copysign(2*M_PI, beta);}

  // Otherwise, move forward and curve to get alpha + beta to zero
  double marker_y_ref = - angular_gain_ * (alpha + beta) * (marker_pose.position.x - distance_to_dock_);
  geometry_msgs::msg::Twist vel_cmd;
  vel_cmd.linear.x = speed_;
  vel_cmd.angular.z = - linear_gain_ * (marker_y_ref - marker_pose.position.y);
  vel_pub_->publish(vel_cmd);

  return Status::RUNNING;
}

// Collision checker
bool ApproachDock::isCollisionFree()
{
  // Expected displacement within simulate_ahead_time
  double sim_position_change;
  int cycle_count = 0;
  static int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  
  // Current pose
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_);
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  // For every expected next step within simulate_ahead time, check collision
  bool fetch_data = true;
  geometry_msgs::msg::Pose2D sim_pose2d;
  while (cycle_count < max_cycle_count) {
    sim_position_change = speed_ * (cycle_count / this->cycle_frequency_);
    sim_pose2d.x = pose2d.x + sim_position_change * cos(pose2d.theta);
    sim_pose2d.y = pose2d.y + sim_position_change * sin(pose2d.theta);
    cycle_count++;

    if (!this->collision_checker_->isCollisionFree(sim_pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace smart_diffbot_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smart_diffbot_navigation::ApproachDock, nav2_core::Behavior)