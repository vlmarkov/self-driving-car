#pragma once

#include <string>
#include <vector>

struct DigitalValues {
    bool forward{0};
    bool backward{0};
    bool left_turn{0};
    bool right_turn{0};
    int timeout_ms{0};
    std::string command{};
};

std::vector<DigitalValues> convert_to_digital_values(float acceleration, float steering);
