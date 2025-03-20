#pragma once

struct MotorCommands {
    bool engine_left_high{true};
    bool engine_left_low{false};
    bool engine_right_high{true};
    bool engine_right_low{false}
    int engine_left_voltage{0};
    int engine_right_voltage{0};
};

MotorCommands get_motor_commands(int is_stop, int is_forward, int is_left_turn);
