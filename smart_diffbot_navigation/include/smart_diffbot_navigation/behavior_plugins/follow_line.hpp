#ifndef SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__FOLLOW_LINE_HPP_
#define SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__FOLLOW_LINE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <thread>
#include <algorithm>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "smart_diffbot_msgs/action/follow_line.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace smart_diffbot_navigation
{

using namespace nav2_behaviors; 
using namespace std::chrono_literals;
using std::placeholders::_1;
using FollowLineAction = smart_diffbot_msgs::action::FollowLine;


class FollowLine : public TimedBehavior<FollowLineAction>
{
public:
  FollowLine();
  ~FollowLine();

  Status onRun(const std::shared_ptr<const FollowLineAction::Goal> command) override;

  Status onCycleUpdate() override;
  
  void onConfigure() override;

protected:

  // Collision checker 
  double simulate_ahead_time_;
  bool isCollisionFree();

  // Marker subscription 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr line_midpoint_sub_;
  void line_callback(const geometry_msgs::msg::PoseStamped::SharedPtr);

  // Goal
  double speed_;

  // Control gains
  double control_gain_;

  // Line midpoint position and timing
  double line_x_ = 0.0;
  double line_y_ = 0.0;
  double line_time_ = 0.0;

};

}  // namespace smart_diffbot_navigation

#endif  // SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__FOLLOW_LINE_HPP_