#pragma once

#include <common/base_pub_sub_node.h>

#include <motion-calibration/motion_planner.h>

class MotionCalibration
{
public:
    static constexpr auto kName{"MotionCalibration"};

    MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~MotionCalibration() = default;

    void process_motion_vector();

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
    MotionPlanner motion_planner_;
};
