#pragma once

#include <common/base_pub_sub_node.h>

class ObstacleAvoidance
{
public:
    static constexpr auto kName{"ObstacleAvoidance"};

    ObstacleAvoidance(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~ObstacleAvoidance() = default;

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
