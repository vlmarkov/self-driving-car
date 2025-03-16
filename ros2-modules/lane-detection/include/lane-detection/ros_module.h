#pragma once

#include <common/base_pub_sub_node.h>

#include "lane_detection_module.h"

class LaneDetection
{
public:
    static constexpr auto kName{"LaneDetection"};

    LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~LaneDetection() = default;

    void process_frame(cv::Mat frame);

private:
    LaneDetectionModule impl_;
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
