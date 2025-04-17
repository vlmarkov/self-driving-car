#include <motion-calibration/digital_values.h>

#include <cmath>

namespace {

DigitalValues create_stop(int timeout_ms) {
    DigitalValues dv;
    dv.command = "stop";
    dv.timeout_ms = timeout_ms;

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

size_t steering_to_cmd(double sterring) {
    if (sterring == 0.0) {
        return 0;
    }

    return static_cast<size_t>(fabs(sterring / CALIBRATED_ANGLE));
}

} // namespace

std::vector<DigitalValues> convert_to_digital_values(double acceleration, double steering) {
    std::vector<DigitalValues> result;

    // Split turn radius to discrete turn commands sequence
    if (steering > 0.0) {
        result.push_back(create_stop(BETWEEN_TURN_TIMEOUT_MS));
        for (size_t i = 0; i < steering_to_cmd(steering); ++i) {
            result.push_back(create_right());
            result.push_back(create_stop(BETWEEN_TURN_TIMEOUT_MS));
        }
    } else if (steering < 0.0) {
        result.push_back(create_stop(BETWEEN_TURN_TIMEOUT_MS));
        for (size_t i = 0; i < steering_to_cmd(steering); ++i) {
            result.push_back(create_left());
            result.push_back(create_stop(BETWEEN_TURN_TIMEOUT_MS));
        }
    }

    if (acceleration > 0.0) {
        result.push_back(create_forward());
    } else if (acceleration < 0.0) {
        result.push_back(create_backward());
    }

    if (acceleration == 0.0 && steering == 0.0) {
        result.push_back(create_stop(DEFAULT_TIMEOUT_MS));
    }

    return result;
}
