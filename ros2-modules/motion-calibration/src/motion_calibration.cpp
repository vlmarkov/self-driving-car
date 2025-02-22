#include <motion-calibration/motion_calibration.h>

MotionCalibration::MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}
