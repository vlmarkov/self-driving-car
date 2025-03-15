#pragma once

#include "i_socket.h"

#include <memory>

class TcpClient
{
public:
    TcpClient(std::unique_ptr<ISocket> socket, std::string address, size_t port);
    ~TcpClient();

    void send_command(Command cmd);

private:
    std::unique_ptr<ISocket> socket_;
};
