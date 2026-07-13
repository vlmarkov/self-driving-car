#pragma once

#include <cstdint>

constexpr auto HIGH_SIGNAL = true;
constexpr auto LOW_SIGNAL = false;

constexpr uint32_t PWM_NONE = 0;

struct ChasisCommands {
    bool engine_left_forward{LOW_SIGNAL};
    bool engine_left_reverse{LOW_SIGNAL};

    bool engine_right_forward{LOW_SIGNAL};
    bool engine_right_reverse{LOW_SIGNAL};

    uint32_t engine_left_pwm{PWM_NONE};
    uint32_t engine_right_pwm{PWM_NONE};
};
