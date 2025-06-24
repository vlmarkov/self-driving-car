#include <motion-calibration/motion_planner.h>

#include <string>
#include <iostream>

namespace {

MotorCommands foward_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = LOW_SIGNAL;
    mc.engine_left_reverse  = HIGH_SIGNAL;
    mc.engine_right_forward = LOW_SIGNAL;
    mc.engine_right_reverse = HIGH_SIGNAL;

    mc.engine_left_pwm = PWM_DEFAULT;
    mc.engine_right_pwm = PWM_DEFAULT;

    return mc;
}

MotorCommands backward_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = HIGH_SIGNAL;
    mc.engine_left_reverse  = LOW_SIGNAL;
    mc.engine_right_forward = HIGH_SIGNAL;
    mc.engine_right_reverse = LOW_SIGNAL;

    mc.engine_left_pwm = PWM_DEFAULT;
    mc.engine_right_pwm = PWM_DEFAULT;

    return mc;
}

MotorCommands left_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = HIGH_SIGNAL; // backward
    mc.engine_left_reverse  = LOW_SIGNAL;  // backward
    mc.engine_right_forward = LOW_SIGNAL;  // forward
    mc.engine_right_reverse = HIGH_SIGNAL; // forward

    mc.engine_left_pwm = PWM_DEFAULT;
    mc.engine_right_pwm = PWM_DEFAULT;

    return mc;
}

MotorCommands right_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = LOW_SIGNAL;  // forward
    mc.engine_left_reverse  = HIGH_SIGNAL; // forward
    mc.engine_right_forward = HIGH_SIGNAL; // backward
    mc.engine_right_reverse = LOW_SIGNAL;  // backward

    mc.engine_left_pwm = PWM_DEFAULT;
    mc.engine_right_pwm = PWM_DEFAULT;

    return mc;
}

Direction acceleration2direction(const double acceleration)
{
    if (acceleration > 0.0)
        return Direction::FORWARD;
    if (acceleration < 0.0)
        return Direction::BACKWARD;

    return Direction::NONE;
}

Direction steering2direction(const double steering)
{
    if (steering > 0.0)
        return Direction::RIGHT;
    if (steering < 0.0)
        return Direction::LEFT;

    return Direction::NONE;
}

std::string to_str(const State s) {
    switch (s)
    {
        case State::INVALID: return "INVALID";
        case State::STOP: return "STOP";
        case State::DECREASE_SPEED: return "DECREASE_SPEED";
        case State::INCREASE_SPEED: return "INCREASE_SPEED";
        case State::MAINTAIN_SPEED: return "MAINTAIN_SPEED";
    }
    return "";
}

std::string to_str(const Direction d) {
    switch (d)
    {
        case Direction::NONE: return "NONE";
        case Direction::FORWARD: return "FORWARD";
        case Direction::BACKWARD: return "BACKWARD";
        case Direction::LEFT: return "LEFT";
        case Direction::RIGHT: return "RIGHT";
        case Direction::INVALID: return "INVALID";
    }
    return "";
}

} // namespace

MotionPlanner::MotionPlanner(State state, Direction direction, uint32_t engine_left_pwm,
                             uint32_t engine_right_pwm, uint32_t change_pwm_counter)
    : current_state_(state)
    , current_direction_(direction)
    , engine_left_pwm_(engine_left_pwm)
    , engine_right_pwm_(engine_right_pwm)
    , change_pwm_counter_(change_pwm_counter)
{
    transitions_ = {
        // From STOP state
        {State::STOP, Direction::NONE, {Direction::FORWARD}, State::INCREASE_SPEED, Direction::FORWARD},
        {State::STOP, Direction::NONE, {Direction::BACKWARD}, State::INCREASE_SPEED, Direction::BACKWARD},
        {State::STOP, Direction::NONE, {Direction::LEFT}, State::INCREASE_SPEED, Direction::LEFT},
        {State::STOP, Direction::NONE, {Direction::RIGHT}, State::INCREASE_SPEED, Direction::RIGHT},

        // From INCREASE_SPEED state
        {State::INCREASE_SPEED, Direction::FORWARD, {Direction::FORWARD}, State::MAINTAIN_SPEED, Direction::FORWARD},
        {State::INCREASE_SPEED, Direction::BACKWARD, {Direction::BACKWARD}, State::MAINTAIN_SPEED, Direction::BACKWARD},
        {State::INCREASE_SPEED, Direction::LEFT, {Direction::LEFT}, State::MAINTAIN_SPEED, Direction::LEFT},
        {State::INCREASE_SPEED, Direction::RIGHT, {Direction::RIGHT}, State::MAINTAIN_SPEED, Direction::RIGHT},

        {State::INCREASE_SPEED, Direction::FORWARD, {Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::FORWARD},
        {State::INCREASE_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::BACKWARD},
        {State::INCREASE_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::LEFT},
        {State::INCREASE_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::NONE}, State::DECREASE_SPEED, Direction::RIGHT},

        // From MAINTAIN_SPEED state
        {State::MAINTAIN_SPEED, Direction::FORWARD, {Direction::FORWARD}, State::MAINTAIN_SPEED, Direction::FORWARD},
        {State::MAINTAIN_SPEED, Direction::BACKWARD, {Direction::BACKWARD}, State::MAINTAIN_SPEED, Direction::BACKWARD},
        {State::MAINTAIN_SPEED, Direction::LEFT, {Direction::LEFT}, State::MAINTAIN_SPEED, Direction::LEFT},
        {State::MAINTAIN_SPEED, Direction::RIGHT, {Direction::RIGHT}, State::MAINTAIN_SPEED, Direction::RIGHT},

        {State::MAINTAIN_SPEED, Direction::FORWARD, {Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::FORWARD},
        {State::MAINTAIN_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::BACKWARD},
        {State::MAINTAIN_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::RIGHT, Direction::NONE}, State::DECREASE_SPEED, Direction::LEFT},
        {State::MAINTAIN_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::NONE}, State::DECREASE_SPEED, Direction::RIGHT},

        // From DECREASE_SPEED state
        {State::DECREASE_SPEED, Direction::FORWARD, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::STOP, Direction::NONE},
        {State::DECREASE_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::STOP, Direction::NONE},
        {State::DECREASE_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::STOP, Direction::NONE},
        {State::DECREASE_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT, Direction::NONE}, State::STOP, Direction::NONE},
    };
}

MotorCommands MotionPlanner::do_plan(const double acceleration, const double steering)
{
    // Simple hierachy for line holding procedure
    auto next_direction = acceleration2direction(acceleration);
    if (steering != 0.0)
        next_direction = steering2direction(steering);

    do_transition(next_direction);

    MotorCommands mc;
    switch (current_direction_)
    {
        case Direction::FORWARD:
            mc = foward_command();
            break;
        case Direction::BACKWARD:
            mc = backward_command();
            break;
        case Direction::LEFT:
            mc = left_command();
            break;
        case Direction::RIGHT:
            mc = right_command();
            break;
        default:
            break;
    }

    if (current_state_ == State::INCREASE_SPEED) {
        engine_left_pwm_ += PWM_STEP;
        engine_right_pwm_ += PWM_STEP;
    } else if (current_state_ == State::DECREASE_SPEED) {
        engine_left_pwm_ -= PWM_STEP;
        engine_right_pwm_ -= PWM_STEP;
    }

    mc.engine_left_pwm = engine_left_pwm_;
    mc.engine_right_pwm = engine_right_pwm_;

    std::cout << "engine_left_pwm   : " << int(mc.engine_left_pwm) << std::endl;
    std::cout << "engine_right_pwm  : " << int(mc.engine_right_pwm) << std::endl;
    std::cout << "current state     : " << to_str(current_state_) << std::endl;
    std::cout << "current direction : " << to_str(current_direction_) << std::endl;

    return mc;
}

void MotionPlanner::do_transition(Direction next_direction)
{
    for (const auto& t : transitions_) {
        if (t.current_state == current_state_ &&
            t.current_direction == current_direction_ &&
            t.next_directions.contains(next_direction))
        {
            if (current_state_ == State::INCREASE_SPEED || t.next_state == State::INCREASE_SPEED) {
                change_pwm_counter_ += PWM_STEP;
            }
            if (current_state_ == State::INCREASE_SPEED && t.next_state == State::MAINTAIN_SPEED) {
                if (change_pwm_counter_ != PWM_MAX + PWM_STEP) {
                    return; // Early exit to prevent transition and complete increase speed command
                }
                change_pwm_counter_ = PWM_DEFAULT;
            }

            if (current_state_ == State::DECREASE_SPEED || t.next_state == State::DECREASE_SPEED) {
                change_pwm_counter_ += PWM_STEP;
            }
            if (current_state_ == State::DECREASE_SPEED && t.next_state == State::STOP) {
                if (change_pwm_counter_ != PWM_MAX + PWM_STEP) {
                    return; // Early exit to prevent transition and complete decrease speed command
                }
                change_pwm_counter_ = PWM_DEFAULT;
            }

            current_state_ = t.next_state;
            current_direction_ = t.next_direction;

            return;
        }
    }
}
