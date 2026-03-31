#pragma once

#include <nlohmann/json.hpp>

#include <string>

constexpr auto EXIT = "exit";

constexpr auto RUN_TEST_CMD = "run_test_cmd";

constexpr auto BUILD_SW = "build_sw";
constexpr auto UPDATE_SW = "update_sw";
constexpr auto START_SW = "start_sw";
constexpr auto STOP_SW = "stop_sw";
constexpr auto RESTART_SW = "restart_sw";
constexpr auto START_AUTO_PILOT = "start_auto_pilot";
constexpr auto STOP_AUTO_PILOT = "stop_auto_pilot";
constexpr auto REMOTE_CONTROL = "remote_control";

constexpr auto FORWARD = "w";
constexpr auto BACKWARD = "s";
constexpr auto LEFT = "a";
constexpr auto RIGHT = "d";

constexpr auto SPEED = "speed";
constexpr auto ANGLE = "angle";
constexpr auto DELAY_MS = "delay_ms";

constexpr auto DELIMITER = ':';

struct SystemState
{
    bool is_sw_running{false};
    bool is_auto_pilot_on{false};
};

struct RemoteControlCommand {
    bool is_auto_pilot_on{false};
    float acceleration{0.0};
    float steering{0.0};
};

using SystemCommand = std::string;

class Request {
public:
    explicit Request(std::string str);
    explicit Request(const int speed, const int angle);

    std::string get_command() const;
    std::string get_args() const;

    std::string serialize() const;

private:
    nlohmann::json json_;
};

class Response {
public:
    Response();
    explicit Response(std::string json_str);

    void set_code(std::string code);
    void set_data(std::string data);

    std::string get_code() const;
    std::string get_data() const;

    std::string serialize() const;

private:
    nlohmann::json json_;
};
