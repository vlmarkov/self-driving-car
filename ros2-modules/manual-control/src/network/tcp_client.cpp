#include <manual-control/network/tcp_client.h>

TcpClient::TcpClient(std::unique_ptr<ISocket> socket, std::string address, size_t port)
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

void TcpClient::send_command(Command cmd) {
    // TODO: what about exception saftyness?
    socket_->send(std::move(cmd));
}
