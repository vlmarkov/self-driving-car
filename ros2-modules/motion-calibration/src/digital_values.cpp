#include <motion-calibration/digital_values.h>

namespace {

constexpr auto DEFAULT_TIMEOUT_MS = 1000;

DigitalValues create_stop() {
    DigitalValues dv;
    dv.command = "stop";
    dv.timeout_ms = DEFAULT_TIMEOUT_MS;

    return dv;
}

DigitalValues create_backward() {
    DigitalValues dv;
    dv.backward = true;
    dv.command = "backward";
    dv.timeout_ms = DEFAULT_TIMEOUT_MS;

    return dv;
}

DigitalValues create_forward() {
    DigitalValues dv;
    dv.forward = true;
    dv.command = "forward";
    dv.timeout_ms = DEFAULT_TIMEOUT_MS;

    return dv;
}

DigitalValues create_right() {
    DigitalValues dv;
    dv.right_turn = true;
    dv.command = "right";
    dv.timeout_ms = DEFAULT_TIMEOUT_MS;

    return dv;
}

DigitalValues create_left() {
    DigitalValues dv;
    dv.left_turn = true;
    dv.command = "left";
    dv.timeout_ms = DEFAULT_TIMEOUT_MS;

    return dv;
}


} // namespace

std::vector<DigitalValues> convert_to_digital_values(double acceleration, double steering) {
    std::vector<DigitalValues> result;

    if (steering > 0.0) {
        result.push_back(create_right());
    } else if (steering < 0.0) {
        result.push_back(create_left());
    }

    if (acceleration > 0.0) {
        result.push_back(create_forward());
    } else if (acceleration < 0.0) {
        result.push_back(create_backward());
    }

    if (acceleration == 0.0 && steering == 0.0) {
        result.push_back(create_stop());
    }

    return result;
}
