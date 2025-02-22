#pragma once

#include <common/base_pub_sub_node.h>

class ManualControl
{
public:
    static constexpr auto kName{"ManualControl"};

    ManualControl(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~ManualControl() = default;

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
