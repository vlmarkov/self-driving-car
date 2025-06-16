#include <motion-calibration/motion_planner.h>

namespace {

MotorCommands foward_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = LOW_SIGNAL;
    mc.engine_left_reverse  = HIGH_SIGNAL;
    mc.engine_right_forward = LOW_SIGNAL;
    mc.engine_right_reverse = HIGH_SIGNAL;

    mc.engine_left_pwm = DEFAULT_PWM;
    mc.engine_right_pwm = DEFAULT_PWM;

    return mc;
}

MotorCommands backward_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = HIGH_SIGNAL;
    mc.engine_left_reverse  = LOW_SIGNAL;
    mc.engine_right_forward = HIGH_SIGNAL;
    mc.engine_right_reverse = LOW_SIGNAL;

    mc.engine_left_pwm = DEFAULT_PWM;
    mc.engine_right_pwm = DEFAULT_PWM;

    return mc;
}

MotorCommands left_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = HIGH_SIGNAL; // backward
    mc.engine_left_reverse  = LOW_SIGNAL;  // backward
    mc.engine_right_forward = LOW_SIGNAL;  // forward
    mc.engine_right_reverse = HIGH_SIGNAL; // forward

    mc.engine_left_pwm = DEFAULT_PWM;
    mc.engine_right_pwm = DEFAULT_PWM;

    return mc;
}

MotorCommands right_command()
{
    MotorCommands mc;

    mc.engine_left_forward  = LOW_SIGNAL;  // forward
    mc.engine_left_reverse  = HIGH_SIGNAL; // forward
    mc.engine_right_forward = HIGH_SIGNAL; // backward
    mc.engine_right_reverse = LOW_SIGNAL;  // backward

    mc.engine_left_pwm = DEFAULT_PWM;
    mc.engine_right_pwm = DEFAULT_PWM;

    return mc;
}

Direction acceleration2direction(double acceleration)
{
    if (acceleration > 0.0)
        return Direction::FORWARD;
    if (acceleration < 0.0)
        return Direction::BACKWARD;
    return Direction::NONE;
}

Direction steering2direction(double steering)
{
    if (steering > 0.0)
        return Direction::RIGHT;
    if (steering < 0.0)
        return Direction::LEFT;
    return Direction::NONE;
}

} // namespace

MotionPlanner::MotionPlanner(State initial_state, Direction initial_direction)
    : current_state_(initial_state)
    , current_direction_(initial_direction)
{
    transitions_ = {
        // From Stop state
        {State::STOP, Direction::NONE, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::INCREASE_SPEED},

        // From INCREASE_SPEED state
        {State::INCREASE_SPEED, Direction::FORWARD, {Direction::FORWARD}, State::MAINTAIN_SPEED},
        {State::INCREASE_SPEED, Direction::BACKWARD, {Direction::BACKWARD}, State::MAINTAIN_SPEED},
        {State::INCREASE_SPEED, Direction::LEFT, {Direction::LEFT}, State::MAINTAIN_SPEED},
        {State::INCREASE_SPEED, Direction::RIGHT, {Direction::RIGHT}, State::MAINTAIN_SPEED},

        {State::INCREASE_SPEED, Direction::FORWARD, {Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::INCREASE_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::LEFT, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::INCREASE_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::INCREASE_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT}, State::DECREASE_SPEED},

        // From MAINTAIN_SPEED state
        {State::MAINTAIN_SPEED, Direction::FORWARD, {Direction::FORWARD}, State::MAINTAIN_SPEED},
        {State::MAINTAIN_SPEED, Direction::BACKWARD, {Direction::BACKWARD}, State::MAINTAIN_SPEED},
        {State::MAINTAIN_SPEED, Direction::LEFT, {Direction::LEFT}, State::MAINTAIN_SPEED},
        {State::MAINTAIN_SPEED, Direction::RIGHT, {Direction::RIGHT}, State::MAINTAIN_SPEED},

        {State::MAINTAIN_SPEED, Direction::FORWARD, {Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::MAINTAIN_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::LEFT, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::MAINTAIN_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::RIGHT}, State::DECREASE_SPEED},
        {State::MAINTAIN_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT}, State::DECREASE_SPEED},

        // From Decreasing state
        {State::DECREASE_SPEED, Direction::FORWARD, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::STOP},
        {State::DECREASE_SPEED, Direction::BACKWARD, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::STOP},
        {State::DECREASE_SPEED, Direction::LEFT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::STOP},
        {State::DECREASE_SPEED, Direction::RIGHT, {Direction::FORWARD, Direction::BACKWARD, Direction::LEFT, Direction::RIGHT}, State::STOP},
    };
}

MotorCommands MotionPlanner::do_plan(double acceleration, double steering)
{
    // TODO: what about forward+left, forward+right complex command?
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
        mc.engine_left_pwm += (max_counter_ * (counter_ - 1));
        mc.engine_right_pwm += (max_counter_ * (counter_ - 1));
    }

    if (current_state_ == State::DECREASE_SPEED) {
        mc.engine_left_pwm -= (max_counter_ * (counter_ + 1));
        mc.engine_right_pwm -= (max_counter_ * (counter_ + 1));
    }

    return mc;
}

void MotionPlanner::do_transition(Direction next_direction)
{
    for (const auto& t : transitions_) {
        if (t.current_state == current_state_ &&
            t.current_direction == current_direction_ &&
            t.next_directions.contains(next_direction))
        {
            if (current_state_ == State::INCREASE_SPEED && t.next_state == State::MAINTAIN_SPEED) {
                counter_ += 1;
                if (counter_ == max_counter_) {
                    current_state_ = State::MAINTAIN_SPEED;
                }
                return;
            }

            if (current_state_ == State::INCREASE_SPEED && t.next_state == State::DECREASE_SPEED) {
                current_state_ = State::DECREASE_SPEED;
                return;
            }

            if (current_state_ == State::MAINTAIN_SPEED && t.next_state == State::DECREASE_SPEED) {
                current_state_ = State::DECREASE_SPEED;
                return;
            }

            if (current_state_ == State::DECREASE_SPEED && t.next_state == State::STOP) {
                counter_ -= 1;
                if (counter_ == 0) {
                    current_state_ = State::STOP;
                    current_direction_ = Direction::NONE;
                }
                return;
            }

            current_state_ = t.next_state;
            current_direction_ = next_direction;
        }
    }
}
