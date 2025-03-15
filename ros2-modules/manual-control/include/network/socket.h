#pragma once

#include "i_socket.h"

#include <memory>

class ClientSocket : public ISocket
{
public:
    ClientSocket() = default;
    explicit ClientSocket(int fd);
    ~ClientSocket();

    void open(std::string address, size_t port) final;
    void close() final;

    Command read() final;
    void send(Command cmd) final;

private:
    int fd_{-1};
};

class ServerSocket : public IServerSocket
{
public:
    ServerSocket() = default;
    ~ServerSocket();

    void open(std::string address, size_t port) final;
    void close() final;

    void listen() final;
    std::unique_ptr<ISocket> accept_client() final;

private:
    int fd_{-1};
};

std::unique_ptr<ISocket> create_client_socket();
std::unique_ptr<IServerSocket> create_server_socket();
