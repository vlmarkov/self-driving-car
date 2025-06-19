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
    MotionPlanner(State initial_state,
                  Direction initial_direction,
                  uint8_t initial_engine_left_pwm,
                  uint8_t initial_engine_right_pwm,
                  uint8_t initial_change_pwm_counter);

    MotorCommands do_plan(double acceleration, double steering);   

private:
    uint8_t engine_left_pwm_{DEFAULT_PWM};
    uint8_t engine_right_pwm_{DEFAULT_PWM};
    uint8_t change_pwm_counter_{0};

    State current_state_{State::STOP};
    Direction current_direction_{Direction::NONE};

    std::vector<Transition> transitions_;

    void do_transition(Direction next_direction);
};
