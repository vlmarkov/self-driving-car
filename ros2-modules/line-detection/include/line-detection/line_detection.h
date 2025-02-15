#pragma once

#include <common/base_publish_node.h>

class LineDetection : public BasePublishNode
{
public:
    static constexpr auto kName{"LineDetection"};

    LineDetection();
    ~LineDetection() = default;
};
