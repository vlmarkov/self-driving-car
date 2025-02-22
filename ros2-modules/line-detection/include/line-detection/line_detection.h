#pragma once

#include <common/base_pub_sub_node.h>

class LineDetection
{
public:
    static constexpr auto kName{"LineDetection"};

    LineDetection(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~LineDetection() = default;

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
