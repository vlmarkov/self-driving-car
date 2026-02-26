#pragma once

#include <string>
#include <memory>

class ISocket
{
public:
    virtual ~ISocket() = default;

    virtual void open(std::string address, uint16_t port) = 0;
    virtual void close() = 0;

    virtual int read_data(std::string& data) = 0;
    virtual int send_data(std::string data) = 0;

protected:
    ISocket() = default;
    explicit ISocket(int fd);
};

class IServerSocket
{
public:
    virtual ~IServerSocket() = default;

    virtual void open(std::string address, uint16_t port) = 0;
    virtual void close() = 0;

    virtual void listen() = 0;
    virtual std::unique_ptr<ISocket> accept_client() = 0;

protected:
    IServerSocket() = default;
};
