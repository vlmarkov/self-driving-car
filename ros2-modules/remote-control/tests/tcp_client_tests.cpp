#include <remote-control/network/tcp_client.h>
#include <remote-control/remote_control_commands.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

constexpr auto TEST_IP = "localhost";
constexpr auto TEST_PORT = 8080;
constexpr auto TEST_REQ_SIZE = 5;
constexpr auto TEST_RES = "ok";
constexpr auto TEST_RES_SIZE = 2;

const auto TEST_REQ = Request("build").serialize();

class MockSocket : public ISocket {
public:
    MOCK_METHOD(void, open, (std::string address, uint16_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(int, read_data, (std::string&), (final));
    MOCK_METHOD(int, send_data, (std::string), (final));
};

} // namespace

TEST(TcpClientTest, ExpectCreation) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*(mock_socket.get()), open(TEST_IP, TEST_PORT)).Times(1);
    EXPECT_CALL(*(mock_socket.get()), close).Times(1);

    TcpClient tcp_client(std::move(mock_socket), TEST_IP, TEST_PORT);
}

TEST(TcpClientTest, ExpectSendRequest) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*(mock_socket.get()), send_data).Times(1).WillOnce(
        ::testing::Return(TEST_REQ_SIZE)
    );

    TcpClient tcp_client(std::move(mock_socket), TEST_IP, TEST_PORT);

    EXPECT_EQ(tcp_client.send_request(TEST_REQ), TEST_REQ_SIZE);
}

TEST(TcpClientTest, ExpectReadResponse) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*(mock_socket.get()), read_data).Times(1).WillOnce(
        [](std::string& s) { s = TEST_RES; return TEST_RES_SIZE; }
    );

    TcpClient tcp_client(std::move(mock_socket), TEST_IP, TEST_PORT);

    std::string response;
    EXPECT_EQ(tcp_client.read_response(response), TEST_RES_SIZE);
    EXPECT_EQ(response, TEST_RES);
}
