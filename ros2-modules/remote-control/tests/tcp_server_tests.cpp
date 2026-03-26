#include <remote-control/network/tcp_server.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

constexpr auto TEST_IP = "localhost";
constexpr auto TEST_PORT = 8080;
constexpr auto TEST_RES = "ok";
constexpr auto TEST_RES_SIZE = 2;
constexpr auto TEST_REQ = "build";
constexpr auto TEST_REQ_SIZE = 5;

class MockServerSocket : public IServerSocket {
public:
    MOCK_METHOD(void, open, (std::string address, uint16_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(void, listen, (), (final));
    MOCK_METHOD(std::unique_ptr<ISocket>, accept_client, (), (final));
};

class MockClientSocket : public ISocket {
public:
    MOCK_METHOD(void, open, (std::string address, uint16_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(bool, is_connected, (), (final));
    MOCK_METHOD(int, read_data, (std::string&), (final));
    MOCK_METHOD(int, send_data, (std::string), (final));
};

} // namespace

TEST(TcpServerTest, ExpectCreation) {
    auto mock_server_socket = std::make_unique<MockServerSocket>();
    auto mock_client_socket = std::make_unique<MockClientSocket>();

    EXPECT_CALL(*(mock_client_socket.get()), is_connected).Times(1).WillOnce(
        ::testing::Return(true)
    );

    EXPECT_CALL(*(mock_server_socket.get()), open(TEST_IP, TEST_PORT)).Times(1);
    EXPECT_CALL(*(mock_server_socket.get()), listen).Times(1);
    EXPECT_CALL(*(mock_server_socket.get()), accept_client).Times(1).WillOnce(
        ::testing::Return(std::move(mock_client_socket))
    );
    EXPECT_CALL(*(mock_server_socket.get()), close).Times(1);

    TcpServer tcp_server(std::move(mock_server_socket), TEST_IP, TEST_PORT);
}

TEST(TcpClentTest, ExpectReadRequest) {
    auto mock_server_socket = std::make_unique<MockServerSocket>();
    auto mock_client_socket = std::make_unique<MockClientSocket>();

    EXPECT_CALL(*(mock_client_socket.get()), is_connected).Times(1).WillOnce(
        ::testing::Return(true)
    );
    EXPECT_CALL(*(mock_client_socket.get()), read_data).Times(1).WillOnce(
        [](std::string& s) { s = TEST_REQ; return TEST_REQ_SIZE; }
    );
    EXPECT_CALL(*(mock_server_socket.get()), accept_client).Times(1).WillOnce(
        ::testing::Return(std::move(mock_client_socket))
    );

    TcpServer tcp_server(std::move(mock_server_socket), TEST_IP, TEST_PORT);

    std::string request;
    EXPECT_EQ(tcp_server.read_request(request), TEST_REQ_SIZE);
    EXPECT_EQ(request, TEST_REQ);
}

TEST(TcpClentTest, ExpectSendResponse) {
    auto mock_server_socket = std::make_unique<MockServerSocket>();
    auto mock_client_socket = std::make_unique<MockClientSocket>();

    EXPECT_CALL(*(mock_client_socket.get()), is_connected).Times(1).WillOnce(
        ::testing::Return(true)
    );
    EXPECT_CALL(*(mock_client_socket.get()), send_data).Times(1).WillOnce(
        ::testing::Return(TEST_RES_SIZE)
    );
    EXPECT_CALL(*(mock_server_socket.get()), accept_client).Times(1).WillOnce(
        ::testing::Return(std::move(mock_client_socket))
    );

    TcpServer tcp_server(std::move(mock_server_socket), TEST_IP, TEST_PORT);

    EXPECT_EQ(tcp_server.send_response(TEST_RES), TEST_RES_SIZE);
}
