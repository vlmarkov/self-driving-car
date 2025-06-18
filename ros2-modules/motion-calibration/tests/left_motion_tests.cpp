#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(LeftMotorCommandsTest, FromStopToLeftExpectIncrease) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 10);
}

TEST(LeftMotorCommandsTest, FromStopToLeftExpectMaintain) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::STOP, Direction::NONE);

    for (auto i = 0; i < 10; ++i)
        planner.do_plan(acceleration, steering);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM + 20);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM + 20);
}

TEST(LeftMotorCommandsTest, LeftToForward) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(LeftMotorCommandsTest, LeftToBackward) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(LeftMotorCommandsTest, LeftToLeft) {
    double acceleration = 0.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);
    
    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}

TEST(LeftMotorCommandsTest, LeftToRight) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(LeftMotorCommandsTest, LeftToForwardDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(LeftMotorCommandsTest, LeftToBackwardDecrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}

TEST(LeftMotorCommandsTest, LeftToRightDecrease) {
    double acceleration = 0.0;
    double steering = 1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::LEFT);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM - 10);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM - 10);
}