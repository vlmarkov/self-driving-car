#pragma once

#include <common/base_pub_sub_node.h>

#include "chasiss_control.h"

class ChasisControl
{
public:
    static constexpr auto kName{"chasiss_control"};

    ChasisControl(std::shared_ptr<IPubSubNode> pub_sub_node, const ChasissConfig& cfg);
    ~ChasisControl();

    void process_motion_vector();

private:
    StateMachine state_machine_;
    std::shared_ptr<IPubSubNode> pub_sub_node_;
    ChasissConfig cfg_;
};
