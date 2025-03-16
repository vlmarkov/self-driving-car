#pragma once

#include "command.h"

#include <string>
#include <memory>

class ISocket
{
public:
    virtual ~ISocket() = default;

    virtual void open(std::string address, size_t port) = 0;
    virtual void close() = 0;

    virtual Command read() = 0;
    virtual void send(Command cmd) = 0;

protected:
    ISocket() = default;
    explicit ISocket(int fd);
};

class IServerSocket
{
public:
    virtual ~IServerSocket() = default;

    virtual void open(std::string address, size_t port) = 0;
    virtual void close() = 0;

    virtual void listen() = 0;
    virtual std::unique_ptr<ISocket> accept_client() = 0;

protected:
    IServerSocket() = default;
};

