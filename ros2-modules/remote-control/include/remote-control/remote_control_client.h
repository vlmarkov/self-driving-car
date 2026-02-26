#pragma once

#include <remote-control/network/tcp_client.h>
#include <remote-control/remote_control_commands.h>

#include <optional>

std::optional<Response> process_user_input(
    const std::string& user_input,
    TcpClient& client,
    int& speed,
    int& angle
);
