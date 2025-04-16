#include "motor_commands.h"

MotorCommands get_motor_commands(int binary_command) {
    MotorCommands mc;

    if (binary_command == FORWARD) {
        mc.engine_left_forward  = LOW_SIGNAL;
        mc.engine_left_reverse  = HIGH_SIGNAL;
        mc.engine_right_forward = LOW_SIGNAL;
        mc.engine_right_reverse = HIGH_SIGNAL;

        mc.engine_left_pwm = DEFAULT_PWM;
        mc.engine_right_pwm = DEFAULT_PWM;
    } else if (binary_command == BACKWARD) {
        mc.engine_left_forward  = HIGH_SIGNAL;
        mc.engine_left_reverse  = LOW_SIGNAL;
        mc.engine_right_forward = HIGH_SIGNAL;
        mc.engine_right_reverse = LOW_SIGNAL;

        mc.engine_left_pwm = DEFAULT_PWM;
        mc.engine_right_pwm = DEFAULT_PWM;
    } else if (binary_command == LEFT_TURN) {
        mc.engine_left_forward  = HIGH_SIGNAL; // backward
        mc.engine_left_reverse  = LOW_SIGNAL;  // backward
        mc.engine_right_forward = LOW_SIGNAL;  // forward
        mc.engine_right_reverse = HIGH_SIGNAL; // forward

        mc.engine_left_pwm = TURN_PWM;
        mc.engine_right_pwm = TURN_PWM;
    } else if (binary_command == RIGHT_TURN) {
        mc.engine_left_forward  = LOW_SIGNAL;  // forward
        mc.engine_left_reverse  = HIGH_SIGNAL; // forward
        mc.engine_right_forward = HIGH_SIGNAL; // backward
        mc.engine_right_reverse = LOW_SIGNAL;  // backward

        mc.engine_left_pwm = TURN_PWM;
        mc.engine_right_pwm = TURN_PWM;
    }

    return mc;
}
