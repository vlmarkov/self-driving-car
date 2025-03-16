#include <manual-control/network/tcp_server.h>

TcpServer::TcpServer(std::unique_ptr<IServerSocket> socket, std::string address, size_t port)
    : server_socket_(std::move(socket))
{
    server_socket_->open(address, port);
    server_socket_->listen();

    client_socket_ = server_socket_->accept_client();
}

TcpServer::~TcpServer() {
    try {
        client_socket_->close();
        server_socket_->close();
    } catch(...) {
        // Right now I do not care about any error!
    }
}

Command TcpServer::read_command() {
    // TODO: what about exception saftyness?
    return client_socket_->read();
}
