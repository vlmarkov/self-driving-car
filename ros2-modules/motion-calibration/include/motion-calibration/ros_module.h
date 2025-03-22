#pragma once

#include <common/base_pub_sub_node.h>

class MotionCalibration
{
public:
    static constexpr auto kName{"MotionCalibration"};

    MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~MotionCalibration() = default;

    void process_motion_vector();

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
