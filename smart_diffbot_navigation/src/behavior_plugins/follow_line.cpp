#include "smart_diffbot_navigation/behavior_plugins/follow_line.hpp"

using namespace std::chrono_literals;

namespace smart_diffbot_navigation
{

FollowLine::FollowLine()
: TimedBehavior<FollowLineAction>()
{
}

FollowLine::~FollowLine() = default;

void FollowLine::line_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  line_x_ = msg->pose.position.x;
  line_y_ = msg->pose.position.y;
  line_time_ = this->clock_->now().seconds();
}

void FollowLine::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Subscribe to marker pose 
  line_midpoint_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "line_detection/midpoint", 10, std::bind(&FollowLine::line_callback, this, _1));

  // Get parameters
  nav2_util::declare_parameter_if_not_declared(node, "follow_line.control_gain", rclcpp::ParameterValue(1.0));
  node->get_parameter("follow_line.control_gain", control_gain_);  
  nav2_util::declare_parameter_if_not_declared(node, "follow_line.drive_on_secs", rclcpp::ParameterValue(0.0));
  node->get_parameter("follow_line.drive_on_secs", drive_on_secs_);
  nav2_util::declare_parameter_if_not_declared(node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  nav2_util::declare_parameter_if_not_declared(node, "follow_line.check_collisions", rclcpp::ParameterValue(true));
  node->get_parameter("follow_line.check_collisions", check_collisions_);
}

Status FollowLine::onRun(const std::shared_ptr<const FollowLineAction::Goal> command)
{
  // Extract command
  speed_ = command->speed;

  return Status::SUCCEEDED;
}

Status FollowLine::onCycleUpdate()
{
  // If collision is expected, fail the action
  if (check_collisions_ && !isCollisionFree()) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting line following");
    return Status::FAILED;
  }

  // If line is marked finished, move on in same direction for an amount of seconds, then succeed
  if(line_finished_){
    if(this->clock_->now().seconds() - line_finish_time_ < drive_on_secs_){
      geometry_msgs::msg::Twist vel_cmd;
      vel_cmd.linear.x = speed_;
      vel_pub_->publish(vel_cmd);
      return Status::RUNNING;
    }
    else{
      stopRobot();
      line_finished_ = false;
      return Status::SUCCEEDED;
    }
  }

  // If line not finished yet, check if it is seen recently
  if(line_time_ == 0.0){
    RCLCPP_WARN(logger_, "No line spotted, stopping follow_line action");
    return Status::FAILED;
  }

  if(!line_finished_ && this->clock_->now().seconds() - line_time_ > 1.0){
    RCLCPP_WARN(logger_, "Lost sight of line, stopping follow_line action. Last seen %d seconds ago.", int(this->clock_->now().seconds() - line_time_));
    stopRobot();
    return Status::FAILED;
  }

  // Follow line by adapting angular velocity to line_x (and linear velocity on goal request)
  geometry_msgs::msg::Twist vel_cmd;
  vel_cmd.linear.x = speed_;
  vel_cmd.angular.z = - control_gain_ * line_x_;
  vel_pub_->publish(vel_cmd);

  // Check if line needs to be marked as finished
  if(line_y_ < 20.0){
      line_finished_ = true;
      RCLCPP_INFO(logger_, "Line finished");
      if(drive_on_secs_ != 0.0){
        RCLCPP_INFO(logger_, "Driving on for %d seconds...", int(drive_on_secs_));
      }
      line_finish_time_ = this->clock_->now().seconds();
  }

  return Status::RUNNING;
}

// Collision checker
bool FollowLine::isCollisionFree()
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
PLUGINLIB_EXPORT_CLASS(smart_diffbot_navigation::FollowLine, nav2_core::Behavior)