#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(BackwardMotorCommandsTest, FromStopToBackwardExpectIncrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 10);
}

TEST(BackwardMotorCommandsTest, FromStopToBackwardExpectMaintain) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 20);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 20);
}

TEST(BackwardMotorCommandsTest, BackwardToForward) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(BackwardMotorCommandsTest, BackwardToBackward) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(BackwardMotorCommandsTest, BackwardToLeft) {
    double acceleration = -1.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(BackwardMotorCommandsTest, BackwardToRight) {
    double acceleration = -1.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM -10);
}

TEST(BackwardMotorCommandsTest, BackwardToForwardDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(BackwardMotorCommandsTest, BackwardToLeftDecrease) {
    double acceleration = -1.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(BackwardMotorCommandsTest, BackwardToRightDecrease) {
    double acceleration = -1.0;
    double steering = 1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM -10);
}
