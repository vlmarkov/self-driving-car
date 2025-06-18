#pragma once

#include "motor_commands.h"

#include <unordered_set>
#include <functional>

enum class State {STOP, INCREASE_SPEED, MAINTAIN_SPEED, DECREASE_SPEED };
enum class Direction {NONE, FORWARD, BACKWARD, LEFT, RIGHT };

struct Transition {
    State current_state;
    Direction current_direction;

    std::unordered_set<Direction> next_directions;
    State next_state;
    Direction next_direction;
};


class MotionPlanner {
public:
    MotionPlanner(State initial_state, Direction initial_direction);

    MotorCommands do_plan(double acceleration, double steering);   

private:
    const uint8_t pwm_step{10};
    uint8_t engine_left_pwm = DEFAULT_PWM;
    uint8_t engine_right_pwm = DEFAULT_PWM;

    const uint8_t max_change_pwm_{3};
    uint8_t change_pwm_counter_{0};

    State current_state_{State::STOP};
    Direction current_direction_{Direction::NONE};

    std::vector<Transition> transitions_;

    void do_transition(Direction next_direction);
};
