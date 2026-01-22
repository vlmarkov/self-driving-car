#pragma once

#include "i_socket.h"

#include <memory>

class TcpClient
{
public:
    TcpClient(std::unique_ptr<ISocket> socket, std::string address, uint16_t port);
    ~TcpClient();

    int send_request(std::string request);
    int read_response(std::string& response);

private:
    std::unique_ptr<ISocket> socket_;
};
