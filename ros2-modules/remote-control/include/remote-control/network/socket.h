#pragma once

#include "i_socket.h"

#include <memory>

constexpr auto MAX_DATA_SIZE = 256;

class ClientSocket : public ISocket
{
public:
    ClientSocket() = default;
    explicit ClientSocket(int fd);
    ~ClientSocket();

    void open(std::string address, uint16_t port) final;
    void close() final;

    bool is_connected() final;

    int read_data(std::string& data) final;
    int send_data(std::string data) final;

private:
    int fd_{-1};
};

class ServerSocket : public IServerSocket
{
public:
    ServerSocket() = default;
    ~ServerSocket();

    void open(std::string address, uint16_t port) final;
    void close() final;

    void listen() final;
    std::unique_ptr<ISocket> accept_client() final;

private:
    int fd_{-1};
};

std::unique_ptr<ISocket> create_client_socket();
std::unique_ptr<IServerSocket> create_server_socket();
