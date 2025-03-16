#include <manual-control/ros_module.h>

#include <manual-control/network/command.h>

ManualControl::ManualControl(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}

void ManualControl::process_command(Command cmd) {
    auto mv = pub_sub_node_->get_subscription_msg();

    mv.is_auto_pilot_on = cmd.is_auto_pilot_on;
    mv.acceleration = cmd.acceleration;
    mv.steering = cmd.steering;

    pub_sub_node_->set_publish_msg(std::move(mv));
}
