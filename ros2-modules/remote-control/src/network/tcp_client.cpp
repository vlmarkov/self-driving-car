#include <remote-control/network/tcp_client.h>

TcpClient::TcpClient(std::unique_ptr<ISocket> socket, std::string address, uint16_t port)
    : socket_(std::move(socket))
{
    socket_->open(address, port);
}

TcpClient::~TcpClient() {
    try {
        socket_->close();
    } catch(...) {
        // Right now I do not care about any error!
    }
}

int TcpClient::send_request(std::string request) {
    return socket_->send_data(std::move(request));
}

int TcpClient::read_response(std::string& response) {
    return socket_->read_data(response);
}
