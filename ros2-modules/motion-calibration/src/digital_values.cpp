#include <motion-calibration/digital_values.h>

std::vector<DigitalValues> convert_to_digital_values(float acceleration, float steering) {
    std::vector<DigitalValues> result;
    auto remain_timeout_ms = 1000;

    if (steering > 0.0) {
        // TODO: angle to delay!
        remain_timeout_ms -= 500;

        DigitalValues dv;
        dv.right_turn = true;
        dv.command = "right";
        dv.timeout_ms = remain_timeout_ms;
        result.push_back(dv);
    } else if (steering < 0.0) {
        // TODO: angle to delay!
        remain_timeout_ms -= 500;

        DigitalValues dv;
        dv.left_turn = true;
        dv.command = "left";
        dv.timeout_ms = remain_timeout_ms;
        result.push_back(dv);
    }

    if (acceleration > 0.0) {
        DigitalValues dv;
        dv.forward = true;
        dv.command = "forward";
        dv.timeout_ms = remain_timeout_ms;
        result.push_back(dv);
    } else if (acceleration < 0.0) {
        DigitalValues dv;
        dv.backward = true;
        dv.command = "backward";
        dv.timeout_ms = remain_timeout_ms;
        result.push_back(dv);
    } else {
        DigitalValues dv;
        dv.command = "stop";
        dv.timeout_ms = remain_timeout_ms;
        result.push_back(dv);
    }

    return result;
}
