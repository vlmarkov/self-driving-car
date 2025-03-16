#pragma once

#include "i_socket.h"

#include <memory>

class TcpServer
{
public:
    TcpServer(std::unique_ptr<IServerSocket> socket, std::string address, size_t port);
    ~TcpServer();

    Command read_command();

private:
    std::unique_ptr<IServerSocket> server_socket_;
    std::unique_ptr<ISocket> client_socket_;
};
