#include <manual-control/network/tcp_server.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

constexpr auto TEST_IP = "localhost";
constexpr auto TEST_PORT = 8080;
constexpr Command TEST_CMD{.is_auto_pilot_on = true, .acceleration = 50.0, .steering = 60.0};

} // namespace

class MockServerSocket : public IServerSocket {
public:
    MOCK_METHOD(void, open, (std::string address, size_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(void, listen, (), (final));
    MOCK_METHOD(std::unique_ptr<ISocket>, accept_client, (), (final));
};

class MockClientSocket : public ISocket {
    public:
        MOCK_METHOD(void, open, (std::string address, size_t port), (final));
        MOCK_METHOD(void, close, (), (final));
        MOCK_METHOD(Command, read, (), (final));
        MOCK_METHOD(void, send, (Command), (final));
    };

TEST(TcpServerTest, ExpectCreation) {
    auto mock_server_socket = std::make_unique<MockServerSocket>();
    auto mock_client_socket = std::make_unique<MockClientSocket>();

    EXPECT_CALL(*(mock_server_socket.get()), open(TEST_IP, TEST_PORT)).Times(1);
    EXPECT_CALL(*(mock_server_socket.get()), listen()).Times(1);
    EXPECT_CALL(*(mock_server_socket.get()), accept_client()).Times(1).WillOnce(::testing::Return(std::move(mock_client_socket)));
    EXPECT_CALL(*(mock_server_socket.get()), close()).Times(1);

    TcpServer tcp_server(std::move(mock_server_socket), TEST_IP, TEST_PORT);   
}

TEST(TcpClentTest, ExpectSendCmd) {
    auto mock_server_socket = std::make_unique<MockServerSocket>();
    auto mock_client_socket = std::make_unique<MockClientSocket>();

    EXPECT_CALL(*(mock_client_socket.get()), read()).Times(1).WillOnce(::testing::Return(TEST_CMD));
    EXPECT_CALL(*(mock_server_socket.get()), accept_client()).Times(1).WillOnce(::testing::Return(std::move(mock_client_socket)));

    TcpServer tcp_server(std::move(mock_server_socket), TEST_IP, TEST_PORT);   

    EXPECT_EQ(tcp_server.read_command(), TEST_CMD);
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
