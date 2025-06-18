#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(ForwardMotorCommandsTest, FromStopToForwardExpectIncrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 10);
}

TEST(ForwardMotorCommandsTest, FromStopToForwardExpectMaintain) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 20);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 20);
}

TEST(ForwardMotorCommandsTest, ForwardToForward) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(ForwardMotorCommandsTest, ForwardToBackward) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(ForwardMotorCommandsTest, ForwardToLeft) {
    double acceleration = 1.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(ForwardMotorCommandsTest, ForwardToRight) {
    double acceleration = 1.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(ForwardMotorCommandsTest, ForwardToBackwardDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(ForwardMotorCommandsTest, ForwardToLeftDecrease) {
    double acceleration = 1.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(ForwardMotorCommandsTest, ForwardToRightDecrease) {
    double acceleration = 1.0;
    double steering = 1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::FORWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}