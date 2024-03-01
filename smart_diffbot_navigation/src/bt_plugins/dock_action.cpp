#include "smart_diffbot_navigation/bt_plugins/dock_action.hpp"

namespace smart_diffbot_navigation
{

DockAction::DockAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<smart_diffbot_msgs::action::Dock>(xml_tag_name, action_name, conf)
{
  double speed;
  getInput("speed", speed);
  double distance;
  getInput("distance", distance);
  int marker_id;
  getInput("marker_id", marker_id);


  goal_.speed = speed;
  goal_.distance = distance;
  goal_.marker_id = marker_id;
}

void DockAction::on_tick()
{

}

BT::NodeStatus DockAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace smart_diffbot_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<smart_diffbot_navigation::DockAction>(name, "dock", config);
    };

  factory.registerBuilder<smart_diffbot_navigation::DockAction>("Dock", builder);
}