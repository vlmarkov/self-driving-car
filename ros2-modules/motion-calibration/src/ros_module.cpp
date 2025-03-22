#include <motion-calibration/ros_module.h>

#include <motion-calibration/pins.h>
#include <motion-calibration/digital_values.h>

#include <fmt/format.h>

#include <wiringPi.h>

MotionCalibration::MotionCalibration(std::shared_ptr<IPubSubNode> pub_sub_node)
    : pub_sub_node_(pub_sub_node)
{
    //wiringPiSetup();
    //pinMode(FORWARD_PIN, OUTPUT);
    //pinMode(BACKWARD_PIN, OUTPUT);
    //pinMode(LEFT_TURN_PIN, OUTPUT);
    //pinMode(RIGHT_TURN_PIN, OUTPUT);
}

void MotionCalibration::process_motion_vector() {
    const auto mv = pub_sub_node_->get_subscription_msg();
    const auto digital_values = convert_to_digital_values(mv.acceleration, mv.steering);

    for (const auto& dv : digital_values) {
        pub_sub_node_->log(fmt::format("{} command", dv.command.c_str()));
        //digitalWrite(FORWARD_PIN, dv.pin_forward);
        //digitalWrite(BACKWARD_PIN, dv.pin_backward);
        //digitalWrite(LEFT_TURN_PIN, dv.pin_left_turn);
        //digitalWrite(RIGHT_TURN_PIN, dv.pin_right_turn);
        std::this_thread::sleep_for(std::chrono::milliseconds(dv.timeout_ms));
    }
}
