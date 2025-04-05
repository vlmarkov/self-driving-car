#pragma once

// Motors A, Left side
constexpr auto ENGINE_LEFT_PWM_PIN = 5;      // ENA - Enable and PWM
constexpr auto ENGINE_LEFT_REVERSE_PIN = 6;  // IN1 - Reverse Drive
constexpr auto ENGINE_LEFT_FORWARD_PIN = 7;  // IN2 - Forward Drive

// Motors B, Right side
constexpr auto ENGINE_RIGHT_FORWARD_PIN = 8; // IN3 - Forward Drive
constexpr auto ENGINE_RIGHT_REVERSE_PIN = 9; // IN4 - Reverse Drive
constexpr auto ENGINE_RIGHT_PWM_PIN = 10;    // ENB - Enable and PWM

// Raspberry Pi connection
constexpr auto FROM_RAPSBERRY_PIN_0 = 0; // Actual Raspberry Pi pin 31
constexpr auto FROM_RAPSBERRY_PIN_1 = 1; // Actual Raspberry Pi pin 33
constexpr auto FROM_RAPSBERRY_PIN_2 = 2; // Actual Raspberry Pi pin 35
constexpr auto FROM_RAPSBERRY_PIN_3 = 3; // Actual Raspberry Pi pin 37
