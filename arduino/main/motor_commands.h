#pragma once

/*****************************************************************************/
/* Battery : 12V (fully charged)                                             */
/* PWM     : 150                                                             */
/* DELAY   : 25 ms                                                           */
/* SURFACE : smooth (no additional friction)                                 */
/*                                                                           */
/* With these parameters the car can turn on ~3 degrees                      */
/*****************************************************************************/
constexpr auto TURN_PWM = 150;
/*****************************************************************************/
/* Additional calibration information:                                       */
/* 100 ms 150 pwm 37,5 degree                                                */
/* 50 ms 150 pwm 10 degree 12v                                               */
/* 25 ms 150 pwm 3-5 degree 12v                                              */
/*****************************************************************************/

constexpr auto DEFAULT_PWM = 85;
constexpr auto HIGH_SIGNAL = true;
constexpr auto LOW_SIGNAL = false;

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
