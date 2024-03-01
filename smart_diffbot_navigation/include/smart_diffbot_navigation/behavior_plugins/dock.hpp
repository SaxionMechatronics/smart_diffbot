#ifndef SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__APPROACH_DOCK_HPP_
#define SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__APPROACH_DOCK_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <thread>
#include <algorithm>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "smart_diffbot_msgs/action/dock.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace smart_diffbot_navigation
{

using namespace nav2_behaviors; 
using namespace std::chrono_literals;
using std::placeholders::_1;
using DockAction = smart_diffbot_msgs::action::Dock;


class Dock : public TimedBehavior<DockAction>
{
public:
  Dock();
  ~Dock();

  Status onRun(const std::shared_ptr<const DockAction::Goal> command) override;

  Status onCycleUpdate() override;
  
  void onConfigure() override;

protected:

  // Marker subscription 
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_sub_;
  void marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);

  // Goal 
  double speed_;
  double min_distance_;
  int marker_id_;

  // Markers
  ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr markers_;
  double marker_time_ = 0.0;

// Control gains
  double linear_gain_;

};

}  // namespace smart_diffbot_navigation

#endif  // SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__APPROACH_DOCK_HPP_