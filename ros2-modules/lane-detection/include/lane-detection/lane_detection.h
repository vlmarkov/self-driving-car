#pragma once

#include <common/base_pub_sub_node.h>

class LaneDetection
{
public:
    static constexpr auto kName{"LaneDetection"};

    LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~LaneDetection() = default;

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
