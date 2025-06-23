#pragma once

// See output of gpio read all command to get right pin numbers!

// Motors A, Left side
constexpr auto ENGINE_LEFT_PWM_PIN     = 33;  // L298n ENA - Enable and PWM
constexpr auto ENGINE_LEFT_REVERSE_PIN = 29;  // L298n IN1 - Reverse Drive
constexpr auto ENGINE_LEFT_FORWARD_PIN = 31;  // L298n IN2 - Forward Drive

// Motors B, Right side
constexpr auto ENGINE_RIGHT_FORWARD_PIN = 35; // L298n IN3 - Forward Drive
constexpr auto ENGINE_RIGHT_REVERSE_PIN = 37; // L298n IN4 - Reverse Drive
constexpr auto ENGINE_RIGHT_PWM_PIN     = 32; // L298n ENB - Enable and PWM
