#pragma once

constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;
constexpr int LEFT_TURN = 4;
constexpr int RIGHT_TURN = 8;

struct MotorCommands {
    bool engine_left_high{true};
    bool engine_left_low{false};
    bool engine_right_high{true};
    bool engine_right_low{false};
    int engine_left_voltage{0};
    int engine_right_voltage{0};
};

MotorCommands get_motor_commands(int binary_command);
