#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(RightMotorCommandsTest, FromStopToRightExpectIncrease) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::STOP, Direction::NONE, 0, 0, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(RightMotorCommandsTest, FromRightToRightExpectMaintain) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
}

TEST(RightMotorCommandsTest, FromRightToForwardExpectDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(RightMotorCommandsTest, FromRightToBackwardExpectDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(RightMotorCommandsTest, FromRightToLeftExpectDecrease) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::RIGHT, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(RightMotorCommandsTest, FromIncreaseRightToForwardExpectDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(RightMotorCommandsTest, FromIncreaseRightToBackwardExpectDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(RightMotorCommandsTest, FromIncreaseRightToLeftExpectDecrease) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::RIGHT, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}
