#include <manual-control/network/socket.h>

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

int close_socket(int fd) {
    return ::close(fd);
}
    
int open_client_socket(std::string ip_addr, size_t port) {
    struct hostent* host = gethostbyname(ip_addr.c_str());
    sockaddr_in sendSockAddr;
    ::bzero((char*)&sendSockAddr, sizeof(sendSockAddr));
    sendSockAddr.sin_family = AF_INET;
    sendSockAddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr*)*host->h_addr_list));
    sendSockAddr.sin_port = htons(port);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "Error establishing the server socket" << std::endl;
        return -1;
    }

    if (connect(fd, (sockaddr*) &sendSockAddr, sizeof(sendSockAddr)) < 0) {
        std::cerr << "Error connecting to socket!" << std::endl;
        close_socket(fd);
        return -1;
    }

    return fd;
}

int open_server_socket(std::string ip_addr, size_t port) {
    sockaddr_in servAddr;
    ::bzero((char*)&servAddr, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = inet_addr(ip_addr.c_str());
    servAddr.sin_port = htons(port);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "Error establishing the server socket" << std::endl;
        return -1;
    }

    const auto on = 1; 
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &on, sizeof(on)) < 0) {
        std::cerr << "Error reused addr port" << std::endl;
        close_socket(fd);
        return -1;
    }

    if (bind(fd, (struct sockaddr*) &servAddr, sizeof(servAddr)) < 0) {
        std::cerr << "Error binding socket to address" << std::endl;
        close_socket(fd);
        return -1;
    }

    return fd;
}

size_t send_data(int fd, char* data, size_t size) {
    return ::send(fd, data, size, 0);
}

size_t read_data(int fd, char* data, size_t size) {
    return ::recv(fd, data, size, 0);
}

int listen_socket(int sock_fd, int requests) {
    int status = ::listen(sock_fd, requests);
    if (status == -1) {
        std::cerr << "Error binding socket" << std::endl;
    }
    return status;
}

int accept_new_connection(int sock_fd) {
    sockaddr_in newSockAddr;
    socklen_t newSockAddrSize = sizeof(newSockAddr);
    int new_fd = ::accept(sock_fd, (sockaddr *)&newSockAddr, &newSockAddrSize);
    if (new_fd < 0) {
        std::cerr << "Error accepting request from client!" << std::endl;
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
    close();
}

void ClientSocket::open(std::string address, size_t port) {
    fd_ = open_client_socket(std::move(address), port);
}

void ClientSocket::close() {
    if (fd_ == -1) {
        return;
    }

    close_socket(fd_); 
}

Command ClientSocket::read() {
    Command cmd;

    read_data(fd_, reinterpret_cast<char*>(&cmd), sizeof(cmd));
    return cmd;
}

void ClientSocket::send(Command cmd) {
    send_data(fd_, reinterpret_cast<char*>(&cmd), sizeof(cmd));
}

ServerSocket::~ServerSocket() {
    close();
}

void ServerSocket::open(std::string address, size_t port) {
    fd_ = open_server_socket(std::move(address), port);
}

void ServerSocket::close() {
    if (fd_ == -1) {
        return;
    }

    close_socket(fd_);
}

void ServerSocket::listen() {
    listen_socket(fd_, 10 /*default connections */);
}

std::unique_ptr<ISocket> ServerSocket::accept_client() {
    auto new_fd = accept_new_connection(fd_);
    return std::make_unique<ClientSocket>(new_fd);
}

std::unique_ptr<ISocket> create_client_socket() {
    return std::make_unique<ClientSocket>();
}

std::unique_ptr<IServerSocket> create_server_socket() {
    return std::make_unique<ServerSocket>();
}
