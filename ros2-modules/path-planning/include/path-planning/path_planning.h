#pragma once

#include <common/base_pub_sub_node.h>

class PathPlanning
{
public:
    static constexpr auto kName{"PathPlanning"};

    PathPlanning(std::shared_ptr<IPubSubNode> pub_sub_node);
    ~PathPlanning() = default;

private:
    std::shared_ptr<IPubSubNode> pub_sub_node_;
};
