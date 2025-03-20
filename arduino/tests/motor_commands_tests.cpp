#include "../main/motor_commands.h"

#include <gtest/gtest.h>

namespace {

constexpr auto HIGH = true;
constexpr auto LOW = false;

} // namespace

TEST(MotorCommandsTest, ExpectStop) {
    const auto mc = get_motor_commands(0);

    ASSERT_EQ(mc.engine_left_high, HIGH);
    ASSERT_EQ(mc.engine_left_low, LOW);
    ASSERT_EQ(mc.engine_right_high, HIGH);
    ASSERT_EQ(mc.engine_right_low, LOW);
    ASSERT_EQ(mc.engine_left_voltage, 0);
    ASSERT_EQ(mc.engine_right_voltage, 0);
}

TEST(MotorCommandsTest, ExpectForward) {
    const auto mc = get_motor_commands(FORWARD);

    ASSERT_EQ(mc.engine_left_high, LOW);
    ASSERT_EQ(mc.engine_left_low, HIGH);
    ASSERT_EQ(mc.engine_right_high, LOW);
    ASSERT_EQ(mc.engine_right_low, HIGH);
    ASSERT_EQ(mc.engine_left_voltage, 250);
    ASSERT_EQ(mc.engine_right_voltage, 250);
}

TEST(MotorCommandsTest, ExpectBackward) {
    const auto mc = get_motor_commands(BACKWARD);

    ASSERT_EQ(mc.engine_left_high, HIGH);
    ASSERT_EQ(mc.engine_left_low, LOW);
    ASSERT_EQ(mc.engine_right_high, HIGH);
    ASSERT_EQ(mc.engine_right_low, LOW);
    ASSERT_EQ(mc.engine_left_voltage, 250);
    ASSERT_EQ(mc.engine_right_voltage, 250);
}

TEST(MotorCommandsTest, ExpectLeft) {
    const auto mc = get_motor_commands(LEFT_TURN);

    ASSERT_EQ(mc.engine_left_high, LOW);
    ASSERT_EQ(mc.engine_left_low, HIGH);
    ASSERT_EQ(mc.engine_right_high, LOW);
    ASSERT_EQ(mc.engine_right_low, HIGH);
    ASSERT_EQ(mc.engine_left_voltage, 0);
    ASSERT_EQ(mc.engine_right_voltage, 230);
}

TEST(MotorCommandsTest, ExpectRight) {
    const auto mc = get_motor_commands(RIGHT_TURN);

    ASSERT_EQ(mc.engine_left_high, LOW);
    ASSERT_EQ(mc.engine_left_low, HIGH);
    ASSERT_EQ(mc.engine_right_high, HIGH);
    ASSERT_EQ(mc.engine_right_low, LOW);
    ASSERT_EQ(mc.engine_left_voltage, 230);
    ASSERT_EQ(mc.engine_right_voltage, 0);
}
