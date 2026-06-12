#pragma once

#include <common/base_pub_sub_node.h>

#include "lane_detection_module.h"

class LaneDetection
{
public:
    static constexpr auto kName{"lane_detection"};

    LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node, const LaneDetectionCfg& cfg);
    ~LaneDetection() = default;

    void process_frame(const cv::Mat& frame);

private:
    LaneDetectionModule impl_;
    std::shared_ptr<IPubSubNode> pub_sub_node_;
    float steer_angle_{-360.0};
};
