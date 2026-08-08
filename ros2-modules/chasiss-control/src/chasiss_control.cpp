#include <chasiss-control/chasiss_control.h>

#ifdef WIRING_PI_LIB
#include <wiringPi.h>
#endif // WIRING_PI_LIB

#include <iostream>
#include <thread>

namespace {

ChasisCommands foward_command(const ChasissConfig& cfg)
{
    return {
        .engine_left_forward  = LOW_SIGNAL,
        .engine_left_reverse  = HIGH_SIGNAL,
        .engine_right_forward = LOW_SIGNAL,
        .engine_right_reverse = HIGH_SIGNAL,
        .engine_left_pwm = cfg.pwm_max,
        .engine_right_pwm = cfg.pwm_max
    };
}

ChasisCommands backward_command(const ChasissConfig& cfg)
{
    return {
        .engine_left_forward  = HIGH_SIGNAL,
        .engine_left_reverse  = LOW_SIGNAL,
        .engine_right_forward = HIGH_SIGNAL,
        .engine_right_reverse = LOW_SIGNAL,
        .engine_left_pwm = cfg.pwm_max,
        .engine_right_pwm = cfg.pwm_max
    };
}

ChasisCommands left_command(const ChasissConfig& cfg)
{
    return {
        .engine_left_forward  = HIGH_SIGNAL, // backward
        .engine_left_reverse  = LOW_SIGNAL,  // backward
        .engine_right_forward = LOW_SIGNAL,  // forward
        .engine_right_reverse = HIGH_SIGNAL, // forward
        .engine_left_pwm = cfg.pwm_turn,
        .engine_right_pwm = cfg.pwm_turn
    };
}

ChasisCommands right_command(const ChasissConfig& cfg)
{
    return {
        .engine_left_forward  = LOW_SIGNAL,  // forward
        .engine_left_reverse  = HIGH_SIGNAL, // forward
        .engine_right_forward = HIGH_SIGNAL, // backward
        .engine_right_reverse = LOW_SIGNAL,  // backward
        .engine_left_pwm = cfg.pwm_turn,
        .engine_right_pwm = cfg.pwm_turn
    };
}

} // namespace

void init_motors_pin(const ChasissConfig& cfg) {
#ifdef WIRING_PI_LIB
    wiringPiSetup();

    pinMode(cfg.engine_left_reverse_pin,  OUTPUT);
    pinMode(cfg.engine_left_forward_pin,  OUTPUT);
    pinMode(cfg.engine_right_forward_pin, OUTPUT);
    pinMode(cfg.engine_right_reverse_pin, OUTPUT);

    pinMode(cfg.engine_left_pwm_pin,  PWM_OUTPUT);
    pinMode(cfg.engine_right_pwm_pin, PWM_OUTPUT);
#endif // WIRING_PI_LIB
}

void set_motors_pin(const ChasissConfig& cfg, const ChasisCommands& cc){
#ifdef WIRING_PI_LIB
    digitalWrite(cfg.engine_left_reverse_pin,  cc.engine_left_reverse);
    digitalWrite(cfg.engine_left_forward_pin,  cc.engine_left_forward);
    digitalWrite(cfg.engine_right_forward_pin, cc.engine_right_forward);
    digitalWrite(cfg.engine_right_reverse_pin, cc.engine_right_reverse);

    pwmWrite(cfg.engine_left_pwm_pin,  static_cast<int>(cc.engine_left_pwm));
    pwmWrite(cfg.engine_right_pwm_pin, static_cast<int>(cc.engine_right_pwm));
#endif // WIRING_PI_LIB
}

StopState::StopState(const ChasissConfig& cfg) : cfg_(cfg) {}

EStates StopState::get_state() { return EStates::STOP; }

EStates StopState::do_transaction(const double acceleration, const double steering) {
    if (acceleration == 0 && steering == 0)
        return EStates::STOP;

    return EStates::GET_TRAJECTORY;
}

void StopState::do_action(const double acceleration, const double steering) {
    std::cout << "StopState acceleration " << acceleration << " steering " << steering << std::endl;
    set_motors_pin(cfg_, {});
}

GetTrajectoryState::GetTrajectoryState(const ChasissConfig& cfg) : cfg_(cfg) {}

EStates GetTrajectoryState::get_state() { return EStates::GET_TRAJECTORY; }

EStates GetTrajectoryState::do_transaction(const double acceleration, const double steering) {
    frame_counter_++;
    if (frame_counter_ == cfg_.frames_to_get_trajectory) {
        frame_counter_ = 0;
        return steering != 0 ? EStates::TURN :  EStates::MOVE;
    }

    return EStates::GET_TRAJECTORY;
}

void GetTrajectoryState::do_action(const double acceleration, const double steering) {
    std::cout << "GetTrajectoryState acceleration " << acceleration << " steering " << steering << std::endl;
    set_motors_pin(cfg_, {});
}

TurnState::TurnState(const ChasissConfig& cfg) : cfg_(cfg) {}

EStates TurnState::get_state() { return EStates::TURN; }

EStates TurnState::do_transaction(const double acceleration, const double steering) {
    return EStates::STOP;
}

void TurnState::do_action(const double acceleration, const double steering) {
    std::cout << "TurnState acceleration " << acceleration << " steering " << steering << std::endl;

    set_motors_pin(cfg_, foward_command(cfg_));
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.timeout_before_turn));
    set_motors_pin(cfg_, {});

    for (auto angle = 0;  angle < std::abs(steering); angle += cfg_.engine_turn_angle) {
        if (steering > 0.0) {
            set_motors_pin(cfg_, right_command(cfg_));
        }
        if (steering < 0.0) {
            set_motors_pin(cfg_, left_command(cfg_));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.engine_turn_delay)); // 3 degree
    }

    set_motors_pin(cfg_, {});
}

MoveState::MoveState(const ChasissConfig& cfg) : cfg_(cfg) {}

EStates MoveState::get_state() { return EStates::MOVE; }

EStates MoveState::do_transaction(const double acceleration, const double steering) {
    if ((acceleration_.value() == acceleration) && (steering_.value() == steering)) {
        return EStates::MOVE;
    }

    acceleration_ = std::nullopt;
    steering_ = std::nullopt;
    return EStates::STOP;
}

void MoveState::do_action(const double acceleration, const double steering) {
    std::cout << "MoveState acceleration " << acceleration << " steering " << steering << std::endl;
    acceleration_ = acceleration;
    steering_ = steering;

    if (acceleration > 0.0) {
        set_motors_pin(cfg_, foward_command(cfg_));
        return;
    }

    if (acceleration < 0.0) {
        set_motors_pin(cfg_, backward_command(cfg_));
    }
}

StateMachine::StateMachine(const ChasissConfig& cfg) {
    states_[EStates::STOP] = TransactionInfo{.state_ptr = std::make_shared<StopState>(cfg), .allowed_states = {EStates::STOP, EStates::GET_TRAJECTORY} };
    states_[EStates::GET_TRAJECTORY] = TransactionInfo{.state_ptr = std::make_shared<GetTrajectoryState>(cfg), .allowed_states = {EStates::MOVE, EStates::TURN, EStates::GET_TRAJECTORY}};
    states_[EStates::MOVE] = TransactionInfo{.state_ptr = std::make_shared<MoveState>(cfg), .allowed_states = {EStates::MOVE, EStates::STOP}};
    states_[EStates::TURN] = TransactionInfo{.state_ptr = std::make_shared<TurnState>(cfg), .allowed_states = {EStates::STOP}};

    current_state_ = states_.at(EStates::STOP).state_ptr;
}

void StateMachine::process(const double acceleration, const double steering) {
    auto new_state = current_state_->do_transaction(acceleration, steering);
    do_transaction_(new_state, current_state_->get_state());
    current_state_->do_action(acceleration, steering);
}

void StateMachine::do_transaction_(EStates new_state, EStates current_state) {
    if (states_.at(current_state).allowed_states.count(new_state)) {
        current_state_ = states_.at(new_state).state_ptr;
    } else {
        std::cerr << "Error state" << std::endl;
    }
}
