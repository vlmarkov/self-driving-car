#pragma once

#include <common/base_publish_node.h>

class ManualControl : public BasePublishNode
{
public:
    static constexpr auto kName{"ManualControl"};

    ManualControl();
    ~ManualControl() = default;
};
