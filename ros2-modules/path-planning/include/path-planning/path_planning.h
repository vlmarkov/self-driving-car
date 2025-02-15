#pragma once

#include <common/base_publish_node.h>

class PathPlanning : public BasePublishNode
{
public:
    static constexpr auto kName{"PathPlanning"};

    PathPlanning();
    ~PathPlanning() = default;
};
