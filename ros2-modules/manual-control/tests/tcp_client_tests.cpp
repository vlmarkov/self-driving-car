#include <manual-control/network/tcp_client.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

constexpr auto TEST_IP = "localhost";
constexpr auto TEST_PORT = 8080;
constexpr Command TEST_CMD{.is_auto_pilot_on = true, .acceleration = 50.0, .steering = 60.0};

} // namespace

class MockSocket : public ISocket {
public:
    MOCK_METHOD(void, open, (std::string address, size_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(Command, read, (), (final));
    MOCK_METHOD(void, send, (Command), (final));
};

TEST(TcpClentTest, ExpectCreation) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*(mock_socket.get()), open(TEST_IP, TEST_PORT)).Times(1);
    EXPECT_CALL(*(mock_socket.get()), close()).Times(1);

    TcpClient tcp_client(std::move(mock_socket), TEST_IP, TEST_PORT);
}

TEST(TcpClentTest, ExpectSendCmd) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*(mock_socket.get()), send(TEST_CMD)).Times(1);

    TcpClient tcp_client(std::move(mock_socket), TEST_IP, TEST_PORT);

    tcp_client.send_command(TEST_CMD);    
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
