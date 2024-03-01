#ifndef SMART_DIFFBOT_NAVIGATION__BT_PLUGINS__DOCK_ACTION_HPP_
#define SMART_DIFFBOT_NAVIGATION__BT_PLUGINS__DOCK_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "smart_diffbot_msgs/action/dock.hpp"

namespace smart_diffbot_navigation
{

class DockAction : public nav2_behavior_tree::BtActionNode<smart_diffbot_msgs::action::Dock>
{
  using Action = smart_diffbot_msgs::action::Dock;
  using ActionResult = Action::Result;

public:

  DockAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("speed", 0.1, "Docking speed"),
        BT::InputPort<double>("distance", 1.0, "Distance to dock"),
        BT::InputPort<int>("marker_id", 1, "ID of marker on docking station"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The spin behavior error code")
      });
  }

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

private:
  bool is_recovery_;
};

}  // namespace

#endif  // SMART_DIFFBOT_NAVIGATION__BT_PLUGINS__DOCK_ACTION_HPP_