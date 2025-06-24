#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(ForwardMotorCommandsTest, FromStopToForwardExpectIncrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE, 0, 0, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromForwardToForwardExpectMaintain) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, PWM_MAX, PWM_MAX, PWM_DEFAULT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_MAX);
    ASSERT_EQ(mc.engine_right_pwm, PWM_MAX);
}

TEST(ForwardMotorCommandsTest, FromForwardToBackwardExpectDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, PWM_MAX, PWM_MAX, PWM_DEFAULT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_MAX - PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_MAX - PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromForwardToLeftExpectDecrease) {
    double acceleration = 1.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, PWM_MAX, PWM_MAX, PWM_DEFAULT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_MAX - PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_MAX - PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromForwardToRightExpectDecrease) {
    double acceleration = 1.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, PWM_MAX, PWM_MAX, PWM_DEFAULT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_MAX - PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_MAX - PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromIncreaseForwardToBackwardExpectDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD, 2 * PWM_STEP, 2 * PWM_STEP, PWM_STEP);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromIncreaseForwardToLeftExpectDecrease) {
    double acceleration = 1.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD, 2 * PWM_STEP, 2 * PWM_STEP, PWM_STEP);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(ForwardMotorCommandsTest, FromIncreaseForwardToRightExpectDecrease) {
    double acceleration = 1.0;
    double steering = 1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD, 2 * PWM_STEP, 2 * PWM_STEP, PWM_STEP);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}
