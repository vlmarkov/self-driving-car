#include <remote-control/network/socket.h>

#include <iostream>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>

namespace {

constexpr auto DEFAULT_CONNECTIONS = 1;

int open_client_socket(std::string ip_addr, uint16_t port) {
    struct hostent* host = gethostbyname(ip_addr.c_str());
    sockaddr_in send_sock_addr;
    ::bzero(reinterpret_cast<char*>(&send_sock_addr), sizeof(send_sock_addr));
    send_sock_addr.sin_family = AF_INET;
    send_sock_addr.sin_addr.s_addr = ::inet_addr(inet_ntoa(*reinterpret_cast<struct in_addr*>(*host->h_addr_list)));
    send_sock_addr.sin_port = ::htons(port);

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "[NETWORK]: establish the server socket error!" << std::endl;
        return -1;
    }

    if (::connect(fd, reinterpret_cast<sockaddr*> (&send_sock_addr), sizeof(send_sock_addr)) < 0) {
        std::cerr << "[NETWORK]: connect to socket error!" << std::endl;
        ::close(fd);
        return -1;
    }

    return fd;
}

int open_server_socket(std::string ip_addr, uint16_t port) {
    sockaddr_in serv_addr;
    ::bzero(reinterpret_cast<char*>(&serv_addr), sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = ::inet_addr(ip_addr.c_str());
    serv_addr.sin_port = ::htons(port);

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "[NETWORK]: establish the server socket error!" << std::endl;
        return -1;
    }

    const auto on = 1; 
    if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &on, sizeof(on)) < 0) {
        std::cerr << "[NETWORK]: reused addr port error!" << std::endl;
        ::close(fd);
        return -1;
    }

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&serv_addr), sizeof(serv_addr)) < 0) {
        std::cerr << "[NETWORK]: bind socket to address error!" << std::endl;
        ::close(fd);
        return -1;
    }

    return fd;
}

int listen_socket(int sock_fd, int requests) {
    int status = ::listen(sock_fd, requests);
    if (status == -1) {
        std::cerr << "[NETWORK]: listen socket error!" << std::endl;
    }

    return status;
}

int accept_new_connection(int sock_fd) {
    sockaddr_in new_sock_addr;
    socklen_t new_sock_addr_size = sizeof(new_sock_addr);
    int new_fd = ::accept(sock_fd, reinterpret_cast<sockaddr *>(&new_sock_addr), &new_sock_addr_size);
    if (new_fd < 0) {
        std::cerr << "[NETWORK]: accept client error!" << std::endl;
        return -1;
    }

    return new_fd;
}

} // namespace

ClientSocket::ClientSocket(int fd)
    : fd_(fd)
{
}

ClientSocket::~ClientSocket() {
    ClientSocket::close();
}

void ClientSocket::open(std::string address, uint16_t port) {
    fd_ = open_client_socket(std::move(address), port);
}

void ClientSocket::close() {
    if (fd_ == -1) {
        return;
    }

    ::close(fd_);
}

int ClientSocket::read_data(std::string& data) {
    if (data.size() != MAX_DATA_SIZE) {
        std::cerr << "[NETWORK]: read data error, data size!" << std::endl;
        return -1;
    }

    const auto rc = static_cast<int>(::recv(fd_, reinterpret_cast<void *>(data.data()), data.size(), 0));
    if (rc == -1) {
        std::cerr << "[NETWORK]: read data error, failed to read!" << std::endl;
    } else if (rc == 0) {
        std::cerr << "[NETWORK]: read data error, connection lost!" << std::endl;
    }

    return rc;
}

int ClientSocket::send_data(std::string data) {
    if (data.size() > MAX_DATA_SIZE) {
        std::cerr << "[NETWORK]: send data error, data size!" << std::endl;
        return -1;
    }

    const auto rc = static_cast<int>(::send(fd_, data.c_str(), data.size(), 0));
    if (rc == -1) {
        std::cerr << "[NETWORK]: send data error, failed to send!" << std::endl;
    } else if (rc == 0) {
        std::cerr << "[NETWORK]: send data error, connection lost!" << std::endl;
    }

    return rc;
}

ServerSocket::~ServerSocket() {
    ServerSocket::close();
}

void ServerSocket::open(std::string address, uint16_t port) {
    fd_ = open_server_socket(std::move(address), port);
}

void ServerSocket::close() {
    if (fd_ == -1) {
        return;
    }

    ::close(fd_);
}

void ServerSocket::listen() {
    listen_socket(fd_, DEFAULT_CONNECTIONS);
}

std::unique_ptr<ISocket> ServerSocket::accept_client() {
    return std::make_unique<ClientSocket>(accept_new_connection(fd_));
}

std::unique_ptr<ISocket> create_client_socket() {
    return std::make_unique<ClientSocket>();
}

std::unique_ptr<IServerSocket> create_server_socket() {
    return std::make_unique<ServerSocket>();
}
