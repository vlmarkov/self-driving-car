#pragma once

#include <common/base_publish_node.h>

class ObstacleAvoidance : public BasePublishNode
{
public:
    static constexpr auto kName{"ObstacleAvoidance"};

    ObstacleAvoidance();
    ~ObstacleAvoidance() = default;
};
