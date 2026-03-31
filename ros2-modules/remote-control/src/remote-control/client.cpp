#include <remote-control/remote-control/client.h>
#include <remote-control/network/socket.h>

#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>

namespace {

constexpr auto NAME = "REMOTE-CONTROL-CLIENT";

Response process_request(TcpClient& client, const Request& request) {
    std::string response;
    response.resize(MAX_DATA_SIZE);

    const auto send_bytes = client.send_request(request.serialize());
    if (send_bytes < 1) {
        throw std::runtime_error("lost connection");
    }

    std::cout << "["<< NAME << "] send request "
              << request.serialize()
              << " "
              << send_bytes
              << " bytes"
              << std::endl;

    const auto recv_bytes = client.read_response(response);
    if (send_bytes < 1) {
        throw std::runtime_error("lost connection");
    }

    std::cout << "["<< NAME << "] read response "
              << request.serialize()
              << " "
              << response
              << " "
              << recv_bytes
              << " bytes"
              << std::endl;

    return Response(response);
}

Response process_test_cmd(TcpClient& client, const std::string& file) {
    std::ifstream f(file);
    if (!f.is_open())
        return {};

    process_request(client, Request(START_SW));
    process_request(client, Request(STOP_AUTO_PILOT));

    for (const auto& data: nlohmann::json::parse(f)) {
        const auto speed = data[SPEED].get<int>();
        const auto angle = data[ANGLE].get<int>();
        const auto delay_ms = data[DELAY_MS].get<int>();

        process_request(client, Request(speed, angle));
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    return process_request(client, Request(STOP_AUTO_PILOT));
}

} // namespace

std::optional<Response> process_user_input(
    const std::string& user_input,
    TcpClient& client,
    int& speed,
    int& angle)
{
    const Request request(user_input);
    const auto& command = request.get_command();

    if (command == RESTART_SW) {
        return process_request(client, request);
    } else if (command == BUILD_SW) {
        return process_request(client, request);
    } else if (command == UPDATE_SW) {
        return process_request(client, request);
    } else if (command == START_SW) {
        return process_request(client, request);
    } else if (command == STOP_SW) {
        return process_request(client, request);
    } else if (command == START_AUTO_PILOT) {
        return process_request(client, request);
    } else if (command == STOP_AUTO_PILOT) {
        return process_request(client, request);
    } else if (command == FORWARD) {
        return process_request(client, Request(++speed, angle));
    } else if (command == BACKWARD) {
        return process_request(client, Request(--speed, angle));
    } else if (command == LEFT) {
        return process_request(client, Request(speed, ++angle));
    } else if (command == RIGHT) {
        return process_request(client, Request(speed, --angle));
    } else if (command == RUN_TEST_CMD) {
        auto file = request.get_args();
        if (file.length() >= 2) {
            file.erase(0, 1);
            file.pop_back();
        }
        return process_test_cmd(client, file);
    }

    return {};
}
