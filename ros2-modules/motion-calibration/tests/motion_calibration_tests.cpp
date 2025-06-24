#include <motion-calibration/motion_planner.h>

#include <gtest/gtest.h>

TEST(MotorCommandsTest, ComplexCurveMotion) {
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
        ASSERT_EQ(mc.engine_left_pwm, PWM_MAX);
        ASSERT_EQ(mc.engine_right_pwm, PWM_MAX);
    }

    {
        // MAINTAIN(FORWARD)->DESCREASE(RIGHT)->STOP
        double acceleration = 1.0;
        double steering = -1.0;

        for (auto i = 0; i < 4; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, PWM_DEFAULT);
        ASSERT_EQ(mc.engine_right_pwm, PWM_DEFAULT);
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
        ASSERT_EQ(mc.engine_left_pwm, PWM_MAX);
        ASSERT_EQ(mc.engine_right_pwm, PWM_MAX);
    }

    {
        // MAINTAIN(RIGHT)->DECREASE(LEFT)->STOP
        double acceleration = 1.0;
        double steering = -1.0;

        for (auto i = 0; i < 4; ++i) {
            planner.do_plan(acceleration, steering);
        }

        auto mc = planner.do_plan(acceleration, steering);

        ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
        ASSERT_EQ(mc.engine_left_pwm, PWM_DEFAULT);
        ASSERT_EQ(mc.engine_right_pwm, PWM_DEFAULT);
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
        ASSERT_EQ(mc.engine_left_pwm, PWM_MAX);
        ASSERT_EQ(mc.engine_right_pwm, PWM_MAX);
    }
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
    ASSERT_EQ(mc.engine_left_pwm, PWM_MAX);
    ASSERT_EQ(mc.engine_right_pwm, PWM_MAX);
}

TEST(MotorCommandsTest, FromForwardToStopExpectStop) {
    double acceleration = 0.0;
    double steering = 0.0;

    MotionPlanner planner(State::MAINTAIN_SPEED, Direction::FORWARD, PWM_MAX, PWM_MAX, PWM_DEFAULT);

    for (auto i = 0; i < 10; ++i) {
        planner.do_plan(acceleration, steering);
    }

    auto mc = planner.do_plan(acceleration, steering);

    ASSERT_EQ(mc.engine_left_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_forward, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_right_reverse, LOW_SIGNAL);
    ASSERT_EQ(mc.engine_left_pwm, PWM_DEFAULT);
    ASSERT_EQ(mc.engine_right_pwm, PWM_DEFAULT);
}
