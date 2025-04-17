#pragma once

#include <string>
#include <vector>

/// To get full information about hardware and software details
/// @see: https://github.com/vlmarkov/self-driving-arduino-car/blob/main/arduino/README.md
constexpr auto DEFAULT_TIMEOUT_MS = 25;
constexpr auto BETWEEN_TURN_TIMEOUT_MS = 500;
constexpr auto CALIBRATED_ANGLE = 3.0;

struct DigitalValues {
    bool forward{0};
    bool backward{0};
    bool left_turn{0};
    bool right_turn{0};
    int timeout_ms{0};
    std::string command{};
};

std::vector<DigitalValues> convert_to_digital_values(double acceleration, double steering);
