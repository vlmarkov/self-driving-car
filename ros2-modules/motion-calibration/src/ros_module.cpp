#include <motion-calibration/ros_module.h>

#include <motion-calibration/pins.h>

#include <fmt/format.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

MotionCalibration::MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
    , motion_planner_(State::STOP, Direction::NONE, DEFAULT_PWM, DEFAULT_PWM, 0)
{
#ifdef WIRING_PI_LIB
    wiringPiSetup();

    pinMode(FORWARD_PIN,    OUTPUT);
    pinMode(BACKWARD_PIN,   OUTPUT);
    pinMode(LEFT_TURN_PIN,  OUTPUT);
    pinMode(RIGHT_TURN_PIN, OUTPUT);
#endif // WIRING_PI_LIB
}

void MotionCalibration::process_motion_vector() {
    const auto mv = pub_sub_node_->get_subscription_msg();
    auto mc = motion_planner_.do_plan(mv.acceleration, mv.steering);

    // TODO: add rapsberry + l298 motor dc driver here

#ifdef WIRING_PI_LIB
    digitalWrite(FORWARD_PIN, mc.engine_left_forward);
    digitalWrite(BACKWARD_PIN, mc.engine_left_reverse);
    digitalWrite(LEFT_TURN_PIN, mc.engine_right_forward);
    digitalWrite(RIGHT_TURN_PIN, mc.engine_right_forward);
    // TODO: pwm pins support
#endif // WIRING_PI_LIB

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
