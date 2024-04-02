#include "smart_diffbot_navigation/bt_plugins/follow_line_action.hpp"

namespace smart_diffbot_navigation
{

FollowLineAction::FollowLineAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<smart_diffbot_msgs::action::FollowLine>(xml_tag_name, action_name, conf)
{
  double speed;
  getInput("speed", speed);

  goal_.speed = speed;

}

void FollowLineAction::on_tick()
{

}

BT::NodeStatus FollowLineAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowLineAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowLineAction::on_cancelled()
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
      return std::make_unique<smart_diffbot_navigation::FollowLineAction>(name, "follow_line", config);
    };

  factory.registerBuilder<smart_diffbot_navigation::FollowLineAction>("FollowLine", builder);
}