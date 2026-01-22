#include <remote-control/network/tcp_server.h>

TcpServer::TcpServer(std::unique_ptr<IServerSocket> socket, std::string address, uint16_t port)
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

int TcpServer::send_response(std::string response) {
    return client_socket_->send_data(std::move(response));
}

int TcpServer::read_request(std::string& request) {
    return client_socket_->read_data(request);
}
