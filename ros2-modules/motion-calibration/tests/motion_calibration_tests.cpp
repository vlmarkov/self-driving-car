#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(MotorCommandsTest, ComplexCurveMotion) {
    double acceleration = 0.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE, 0, 0, 0);
    
    {
        // STOP->INCREASE(FORWARD)->MAINTAIN
        double acceleration = 1.0;
        double steering = 0.0;

        for (auto i = 0; i < 10; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
        ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
    }

    {
        // MAINTAIN(FORWARD)->DESCREASE(RIGHT)->STOP
        double acceleration = 1.0;
        double steering = -1.0;

        for (auto i = 0; i < MAX_PWM_STEPS; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
        ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
    }

    {
        // STOP->INCREAESE(RIGHT)->MAINTAIN
        double acceleration = 1.0;
        double steering = 1.0;

        for (auto i = 0; i < 10; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
        ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
    }

    {
        // MAINTAIN(RIGHT)->DECREASE(LEFT)->STOP
        double acceleration = 1.0;
        double steering = -1.0;

        for (auto i = 0; i < MAX_PWM_STEPS; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
        ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
    }

    {
        // STOP->INCREASE(LEFT)->MAINTAIN
        double acceleration = 1.0;
        double steering = -1.0;

        for (auto i = 0; i < 10; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
        ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
    }

    auto motor_commands = planner.do_plan(acceleration, steering);
}

TEST(MotorCommandsTest, FromStopToForwardExpectMaintain) {
    double acceleration = 1.0;
    double steering = 0.0;

    MotionPlanner planner(State::STOP, Direction::NONE, 0, 0, 0);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, HIGH_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, MAX_PWM_STEPS * PWM_STEP);
    ASSERT_EQ(mc.engine_right_pwm, MAX_PWM_STEPS * PWM_STEP);
}

TEST(MotorCommandsTest, FromForwardToStopExpectStop) {
    double acceleration = 0.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, MAX_PWM_STEPS * PWM_STEP, MAX_PWM_STEPS * PWM_STEP, 0);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, DEFAULT_PWM);
    ASSERT_EQ(mc.engine_right_pwm, DEFAULT_PWM);
}
