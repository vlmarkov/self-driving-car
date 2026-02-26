#pragma once

#include "i_socket.h"

#include <memory>

class TcpServer
{
public:
    TcpServer(std::unique_ptr<IServerSocket> socket, std::string address, uint16_t port);
    ~TcpServer();

    int send_response(std::string response);
    int read_request(std::string& request);

private:
    std::unique_ptr<IServerSocket> server_socket_;
    std::unique_ptr<ISocket> client_socket_;
};
