#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(BackwardMotorCommandsTest, FromStopToBackwardExpectIncrease) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE, 0, 0, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromBackwardToBackwardExpectMaintain) {
    double acceleration = -1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromBackwardToForwardExpectDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromBackwardToLeftExpectDecrease) {
    double acceleration = -1.0;
    double steering = -1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromBackwardToRightExpectDecrease) {
    double acceleration = -1.0;
    double steering = 1.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::BACKWARD, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, (MAX_PWM_STEPS - 1) * PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromIncreaseBackwardToForwardDecrease) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromIncreaseBackwardToLeftDecrease) {
    double acceleration = -1.0;
    double steering = -1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}

TEST(BackwardMotorCommandsTest, FromIncreaseBackwardToRightDecrease) {
    double acceleration = -1.0;
    double steering = 1.0;

    MotionPlanner planner(State::INCREASE_SPEED, Direction::BACKWARD, 2 * PWM_STEP, 2 * PWM_STEP, 1);

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, PWM_STEP);
}
