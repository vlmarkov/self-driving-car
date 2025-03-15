#pragma once

struct Command {
    bool is_auto_pilot_on{false};
    float acceleration{0.0};
    float steering{0.0};

    auto operator<=>(const Command&) const = default;
};
