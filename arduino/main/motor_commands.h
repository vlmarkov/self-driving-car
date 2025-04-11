#pragma once

constexpr auto HIGH_SIGNAL = true;
constexpr auto LOW_SIGNAL = false;
constexpr auto DEFAULT_PWM = 85;
constexpr auto TURN_PWM = 75;

constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;
constexpr int LEFT_TURN = 4;
constexpr int RIGHT_TURN = 8;

struct MotorCommands {
    bool engine_left_forward{LOW_SIGNAL};
    bool engine_left_reverse{LOW_SIGNAL};
    bool engine_right_forward{LOW_SIGNAL};
    bool engine_right_reverse{LOW_SIGNAL};
    int engine_left_pwm{0};
    int engine_right_pwm{0};
};

MotorCommands get_motor_commands(int binary_command);
