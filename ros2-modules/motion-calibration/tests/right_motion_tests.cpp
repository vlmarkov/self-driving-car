#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(RightMotorCommandsTest, FromStopToRightExpectIncrease) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 10);
}

TEST(RightMotorCommandsTest, FromStopToRightExpectMaintain) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 20);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 20);
}

TEST(RightMotorCommandsTest, RightToForward) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(RightMotorCommandsTest, RightToBackward) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(RightMotorCommandsTest, RightToLeft) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(RightMotorCommandsTest, RightToRight) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(RightMotorCommandsTest, RightToForwardDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(RightMotorCommandsTest, RightToBackwardDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(RightMotorCommandsTest, RightToLeftDecrease) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}
