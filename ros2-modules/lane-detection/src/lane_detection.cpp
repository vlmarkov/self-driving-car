#include <lane-detection/lane_detection.h>

LaneDetection::LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}
