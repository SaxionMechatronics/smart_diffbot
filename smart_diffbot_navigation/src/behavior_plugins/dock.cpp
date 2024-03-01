#include "smart_diffbot_navigation/behavior_plugins/dock.hpp"

using namespace std::chrono_literals;

namespace smart_diffbot_navigation
{

Dock::Dock()
: TimedBehavior<DockAction>()
{
}

Dock::~Dock() = default;

void Dock::marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  markers_ = msg;
  marker_time_ = steady_clock_.now().seconds();
}

void Dock::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Subscribe to marker pose 
  marker_sub_ = node->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "aruco_markers", 10, std::bind(&Dock::marker_callback, this, _1));

  // Get parameters
  nav2_util::declare_parameter_if_not_declared(node, "dock.linear_gain", rclcpp::ParameterValue(1.0));
  node->get_parameter("dock.linear_gain", linear_gain_);
}

Status Dock::onRun(const std::shared_ptr<const DockAction::Goal> command)
{
  // Extract command
  speed_ = command->speed;
  marker_id_ = command->marker_id;
  min_distance_ = command->distance;

  return Status::SUCCEEDED;
}

Status Dock::onCycleUpdate()
{
  // Check if marker was seen
  if(marker_time_ == 0.0){
    RCLCPP_WARN(logger_, "No marker spotted, stopping dock action");
    return Status::FAILED;
  }

  // Check if it is the right marker
  if(markers_->marker_ids[0] != marker_id_){
    RCLCPP_WARN(logger_, "Detected wrong marker, stopping dock action. Expected %d, found %ld.", marker_id_, markers_->marker_ids[0]);
    return Status::FAILED;
  }

  // Check if the marker pose is up to date enough (<1 sec)
  if (steady_clock_.now().seconds() - marker_time_ > 1.0){
    RCLCPP_WARN(logger_, "Lost sight of marker, stopping dock action. Last seen %d seconds ago.", int(steady_clock_.now().seconds() - marker_time_));
    return Status::FAILED;
  }

  // Rotate marker to z-up frame
  geometry_msgs::msg::Pose marker_pose;
  marker_pose.position.x = markers_->poses[0].position.z;
  marker_pose.position.y = -markers_->poses[0].position.x;

  // If desired distance to dock is reached, succeed
  if(marker_pose.position.x <= min_distance_){
    stopRobot();
    return Status::SUCCEEDED;
  }

  // Otherwise, move forward and keep marker in center
  geometry_msgs::msg::Twist vel_cmd;
  vel_cmd.linear.x = speed_;
  vel_cmd.angular.z = linear_gain_ * marker_pose.position.y;
  vel_pub_->publish(vel_cmd);

  return Status::RUNNING;
}

}  // namespace smart_diffbot_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smart_diffbot_navigation::Dock, nav2_core::Behavior)