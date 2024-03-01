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
#include "smart_diffbot_msgs/action/approach_dock.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
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
using ApproachDockAction = smart_diffbot_msgs::action::ApproachDock;


class ApproachDock : public TimedBehavior<ApproachDockAction>
{
public:
  ApproachDock();
  ~ApproachDock();

  Status onRun(const std::shared_ptr<const ApproachDockAction::Goal> command) override;

  Status onCycleUpdate() override;
  
  void onConfigure() override;

protected:

  // Collision checker 
  double simulate_ahead_time_;
  bool isCollisionFree();

  // Marker subscription 
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_sub_;
  void marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);

  // Goal 
  double speed_;
  double distance_to_dock_;
  int marker_id_;

  // Markers
  ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr markers_;
  double marker_time_ = 0.0;

  // Control gains
  double linear_gain_;
  double angular_gain_;

};

}  // namespace smart_diffbot_navigation

#endif  // SMART_DIFFBOT_NAVIGATION__BEHAVIOR_PLUGINS__APPROACH_DOCK_HPP_