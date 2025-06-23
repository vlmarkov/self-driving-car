#pragma once

#include "motor_commands.h"

#include <unordered_set>
#include <functional>

enum class State {INVALID, STOP, INCREASE_SPEED, MAINTAIN_SPEED, DECREASE_SPEED };
enum class Direction {INVALID, NONE, FORWARD, BACKWARD, LEFT, RIGHT };

struct Transition {
    State current_state{State::INVALID};
    Direction current_direction{Direction::INVALID};

    std::unordered_set<Direction> next_directions;
    State next_state{State::INVALID};
    Direction next_direction{Direction::INVALID};
};

class MotionPlanner {
public:
    MotionPlanner(State state, Direction direction, uint32_t engine_left_pwm,
                  uint32_t engine_right_pwm, uint32_t change_pwm_counter);

    MotorCommands do_plan(const double acceleration, const double steering);

private:
    void do_transition(Direction next_direction);

    State current_state_;
    Direction current_direction_;

    uint32_t engine_left_pwm_;
    uint32_t engine_right_pwm_;
    uint32_t change_pwm_counter_;

    std::vector<Transition> transitions_;
};
