#pragma once

#include "chasiss_commands.h"

#include <memory>
#include <map>
#include <set>
#include <optional>

struct ChasissConfig {
    /**************************************************************************/
    /* To get actual information about Raspberry Pi5 board pins use:          */
    /* - gpio readall command                                                 */
    /*                                                                        */
    /* DO NOT FORGET TO CONNECT GROUND WIRE!                                  */
    /* Raspberry board physical 6 pin -> L298n gnd pin                        */
    /**************************************************************************/

    // Motors A, left side
    int engine_left_pwm_pin     = 1;  // wPi, GPIO18, PHYS12 -> L298n ENA - Enable and PWM
    int engine_left_reverse_pin = 0;  // wPi, GPIO17, PHYS11 -> L298n IN1 - Reverse Drive
    int engine_left_forward_pin = 2;  // wpi, GPIO27, PHYS13 -> L298n IN2 - Forward Drive

    // Motors B, right side
    int engine_right_forward_pin = 3;  // wPi, GPIO22, PHYS15 -> L298n IN3 - Forward Drive
    int engine_right_reverse_pin = 4;  // wPi, GPIO23, PHYS16 -> L298n IN4 - Reverse Drive
    int engine_right_pwm_pin     = 24; // wPi, GPIO19, PHYS35 -> L298n ENB - Enable and PWM

    uint32_t pwm_medium = 256;
    uint32_t pwm_max = 512;
    uint32_t pwm_turn = 300;

    int timeout_before_turn = 100; // msec
    int engine_turn_delay = 25; // msec for 3 degree
    int engine_turn_angle = 3;
    int frames_to_get_trajectory = 10;
};

void init_motors_pin(const ChasissConfig& cfg);
void set_motors_pin(const ChasissConfig& cfg, const ChasisCommands& cc);

enum class EStates {
    STOP = 0,
    GET_TRAJECTORY = 1,
    MOVE = 2,
    TURN = 3
};

struct IState{
    virtual ~IState() = default;

    virtual EStates get_state() = 0;
    virtual EStates do_transaction(const double acceleration, const double steering) = 0;
    virtual void do_action(const double acceleration, const double steering) = 0;
};

struct StopState : public IState {
    StopState(const ChasissConfig& cfg);
    ~StopState() final = default;

    EStates get_state() final;
    EStates do_transaction(const double acceleration, const double steering) final;
    void do_action(const double acceleration, const double steering) final;

private:
    ChasissConfig cfg_;
};

struct GetTrajectoryState : public IState {
    GetTrajectoryState(const ChasissConfig& cfg);
    ~GetTrajectoryState() final = default;

    EStates get_state() final;
    EStates do_transaction(const double acceleration, const double steering) final;
    void do_action(const double acceleration, const double steering) final;

private:
    int frame_counter_ = 0;
    ChasissConfig cfg_;
};

struct TurnState : public IState {
    TurnState(const ChasissConfig& cfg);
    ~TurnState() final = default;

    EStates get_state() final;
    EStates do_transaction(const double acceleration, const double steering) final;
    void do_action(const double acceleration, const double steering) final;

private:
    ChasissConfig cfg_;
};

struct MoveState : public IState {
    MoveState(const ChasissConfig& cfg);
    ~MoveState() final = default;

    EStates get_state() final;
    EStates do_transaction(const double acceleration, const double steering) final;
    void do_action(const double acceleration, const double steering) final;

private:
    std::optional<int> acceleration_ = std::nullopt;
    std::optional<int> steering_ = std::nullopt;
    ChasissConfig cfg_;
};

struct TransactionInfo {
    std::shared_ptr<IState> state_ptr;
    std::set<EStates> allowed_states;
};

struct StateMachine {
    StateMachine(const ChasissConfig& cfg);
    ~StateMachine() = default;

    void process(const double acceleration, const double steering);

private:
    std::shared_ptr<IState> current_state_;
    std::map<EStates, TransactionInfo> states_;

    void do_transaction_(EStates new_state, EStates current_state);
};
