#include <motion-calibration/ros_module.h>

#include <motion-calibration/pins.h>

#include <fmt/format.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

MotionCalibration::MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
    , motion_planner_(State::STOP, Direction::NONE, PWM_DEFAULT, PWM_DEFAULT, PWM_DEFAULT)
{
#ifdef WIRING_PI_LIB
    wiringPiSetup();

    pinMode(ENGINE_LEFT_REVERSE_PIN,  OUTPUT);
    pinMode(ENGINE_LEFT_FORWARD_PIN,  OUTPUT);
    pinMode(ENGINE_RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(ENGINE_RIGHT_REVERSE_PIN, OUTPUT);

    pinMode(ENGINE_LEFT_PWM_PIN,  PWM_OUTPUT);
    pinMode(ENGINE_RIGHT_PWM_PIN, PWM_OUTPUT);
#endif // WIRING_PI_LIB
}

void MotionCalibration::process_motion_vector() {
    const auto mv = pub_sub_node_->get_subscription_msg();
    const auto mc = motion_planner_.do_plan(mv.acceleration, mv.steering);

#ifdef WIRING_PI_LIB
    digitalWrite(ENGINE_LEFT_REVERSE_PIN,  mc.engine_left_reverse);
    digitalWrite(ENGINE_LEFT_FORWARD_PIN,  mc.engine_left_forward);
    digitalWrite(ENGINE_RIGHT_FORWARD_PIN, mc.engine_right_forward);
    digitalWrite(ENGINE_RIGHT_REVERSE_PIN, mc.engine_right_reverse);

    pwmWrite(ENGINE_LEFT_PWM_PIN,  static_cast<int>(mc.engine_left_pwm));
    pwmWrite(ENGINE_RIGHT_PWM_PIN, static_cast<int>(mc.engine_right_pwm));
#endif // WIRING_PI_LIB

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
