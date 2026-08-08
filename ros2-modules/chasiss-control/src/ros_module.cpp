#include <chasiss-control/ros_module.h>

#include <chasiss-control/chasiss_control.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

ChasisControl::ChasisControl(std::shared_ptr<IPubSubNode> pub_sub_node, const ChasissConfig& cfg)
    : state_machine_(cfg)
    , pub_sub_node_(pub_sub_node)
    , cfg_(cfg)
{
    init_motors_pin(cfg_);
    set_motors_pin(cfg_, {});
}

ChasisControl::~ChasisControl() {
    set_motors_pin(cfg_, {});
}

void ChasisControl::process_motion_vector() {
    const auto mv = pub_sub_node_->get_subscription_msg();

    state_machine_.process(mv.acceleration, mv.steering);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
