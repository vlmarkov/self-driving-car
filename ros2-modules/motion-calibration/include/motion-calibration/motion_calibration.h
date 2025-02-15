#pragma once

#include <common/base_publish_node.h>

class MotionCalibration : public BasePublishNode
{
public:
    static constexpr auto kName{"MotionCalibration"};

    MotionCalibration();
    ~MotionCalibration() = default;
};
