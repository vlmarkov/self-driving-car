#pragma once

#include <cstdint>

constexpr auto HIGH_SIGNAL = true;
constexpr auto LOW_SIGNAL = false;

constexpr uint32_t PWM_DEFAULT = 0;
constexpr uint32_t PWM_MAX = 512;
constexpr uint32_t PWM_STEP = 128;

struct MotorCommands {
    bool engine_left_forward{LOW_SIGNAL};
    bool engine_left_reverse{LOW_SIGNAL};

    bool engine_right_forward{LOW_SIGNAL};
    bool engine_right_reverse{LOW_SIGNAL};

    uint32_t engine_left_pwm{PWM_DEFAULT};
    uint32_t engine_right_pwm{PWM_DEFAULT};
};
