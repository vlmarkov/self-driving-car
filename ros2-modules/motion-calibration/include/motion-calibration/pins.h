#pragma once

// To get actual information about Raspberry Pi5 board pins use:
// - gpio readall command

// Motors A, left side
constexpr auto ENGINE_LEFT_PWM_PIN     = 1;  // wPi, GPIO18, PHYS12 -> L298n ENA - Enable and PWM
constexpr auto ENGINE_LEFT_REVERSE_PIN = 0;  // wPi, GPIO17, PHYS11 -> L298n IN1 - Reverse Drive
constexpr auto ENGINE_LEFT_FORWARD_PIN = 2;  // wpi, GPIO27, PHYS13 -> L298n IN2 - Forward Drive

// Motors B, right side
constexpr auto ENGINE_RIGHT_FORWARD_PIN = 3;  // wPi, GPIO22, PHYS15 -> L298n IN3 - Forward Drive
constexpr auto ENGINE_RIGHT_REVERSE_PIN = 4;  // wPi, GPIO23, PHYS16 -> L298n IN4 - Reverse Drive
constexpr auto ENGINE_RIGHT_PWM_PIN     = 24; // wPi, GPIO19, PHYS35 -> L298n ENB - Enable and PWM

// DO NOT FORGET TO CONNECT GROUND WIRE!
// Raspberry board physical 6 pin -> L298n gnd pin