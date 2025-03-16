#pragma once

#include <common/base_pub_sub_node.h>

struct Command;

class ManualControl
{
public:
    static constexpr auto kName{"ManualControl"};

    ManualControl(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~ManualControl() = default;

    void process_command(Command cmd);

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
