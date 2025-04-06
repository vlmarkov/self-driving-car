#include <motion-calibration/ros_module.h>

#include <motion-calibration/pins.h>
#include <motion-calibration/digital_values.h>

#include <fmt/format.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

MotionCalibration::MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
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
    const auto digital_values = convert_to_digital_values(mv.acceleration, mv.steering);

    for (const auto& dv : digital_values) {
        pub_sub_node_->log(fmt::format("{} command", dv.command.c_str()));

#ifdef WIRING_PI_LIB
        digitalWrite(FORWARD_PIN, dv.forward);
        digitalWrite(BACKWARD_PIN, dv.backward);
        digitalWrite(LEFT_TURN_PIN, dv.left_turn);
        digitalWrite(RIGHT_TURN_PIN, dv.right_turn);
#endif // WIRING_PI_LIB

        std::this_thread::sleep_for(std::chrono::milliseconds(dv.timeout_ms));
    }
}
