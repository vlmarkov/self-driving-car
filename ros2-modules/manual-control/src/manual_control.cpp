#include <manual-control/manual_control.h>

ManualControl::ManualControl(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}
