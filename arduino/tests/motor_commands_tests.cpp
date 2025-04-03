#include "../main/motor_commands.h"

#include <gtest/gtest.h>

TEST(MotorCommandsTest, ExpectStop) {
    const auto mc = get_motor_commands(0);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, 0);
    ASSERT_EQ(mc.engine_right_pwm, 0);
}

TEST(MotorCommandsTest, ExpectForward) {
    const auto mc = get_motor_commands(FORWARD);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(MotorCommandsTest, ExpectBackward) {
    const auto mc = get_motor_commands(BACKWARD);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(MotorCommandsTest, ExpectLeft) {
    const auto mc = get_motor_commands(LEFT_TURN);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, TURN_PWM);
    ASSERT_EQ(mc.engine_right_pwm, TURN_PWM);
}

TEST(MotorCommandsTest, ExpectRight) {
    const auto mc = get_motor_commands(RIGHT_TURN);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, TURN_PWM);
    ASSERT_EQ(mc.engine_right_pwm, TURN_PWM);
}
