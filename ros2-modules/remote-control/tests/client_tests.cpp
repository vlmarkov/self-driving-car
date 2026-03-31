#include <remote-control/remote-control/client.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

class MockSocket : public ISocket {
public:
    MOCK_METHOD(void, open, (std::string address, uint16_t port), (final));
    MOCK_METHOD(void, close, (), (final));
    MOCK_METHOD(bool, is_connected, (), (final));
    MOCK_METHOD(int, read_data, (std::string&), (final));
    MOCK_METHOD(int, send_data, (std::string), (final));
};

class RemoteControlClientTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto mock_socket = std::make_unique<MockSocket>();
        mock_socket_ptr = mock_socket.get();

        EXPECT_CALL(*mock_socket_ptr, is_connected).Times(1).WillOnce(
            ::testing::Return(true)
        );

        tcp_client = std::make_unique<TcpClient>(
            std::move(mock_socket),
            "localhost",
            8080
        );

        EXPECT_CALL(*mock_socket_ptr, read_data).Times(1).WillOnce([](std::string& s) {
            s = Response().serialize(); return 1;
        });
    }

    MockSocket *mock_socket_ptr = nullptr;
    std::unique_ptr<TcpClient> tcp_client;
};

} // namespace

TEST_F(RemoteControlClientTest, WithBuildSwExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(BUILD_SW).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(BUILD_SW, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithUpdateSwExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(UPDATE_SW).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(UPDATE_SW, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithStartSwExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(START_SW).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(START_SW, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithRestartSwExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(RESTART_SW).serialize());
        return 1;
    });


    int speed = 0;
    int angle = 0;

    auto response = process_user_input(RESTART_SW, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithStopSwExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(STOP_SW).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(STOP_SW, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithStartApExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(START_AUTO_PILOT).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(START_AUTO_PILOT, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithStopApExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(STOP_AUTO_PILOT).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(STOP_AUTO_PILOT, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithForwardExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(1, 0).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(FORWARD, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithBackwardExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(-1, 0).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(BACKWARD, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithLeftExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(0, 1).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(LEFT, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST_F(RemoteControlClientTest, WithRightExpectValidRequest) {
    EXPECT_CALL(*mock_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        EXPECT_EQ(s, Request(0, -1).serialize());
        return 1;
    });

    int speed = 0;
    int angle = 0;

    auto response = process_user_input(RIGHT, *(tcp_client.get()), speed, angle);
    EXPECT_TRUE(response.has_value());
}

TEST(RemoteControlClientInvalidRequestTest, ExpectNoRequest) {
    auto mock_socket = std::make_unique<MockSocket>();

    EXPECT_CALL(*mock_socket.get(), read_data).Times(0);
    EXPECT_CALL(*mock_socket.get(), send_data).Times(0);
    EXPECT_CALL(*mock_socket, is_connected).Times(1).WillOnce(
        ::testing::Return(true)
    );

    auto tcp_client = std::make_unique<TcpClient>(
        std::move(mock_socket),
        "localhost",
        8080
    );

    int speed = 0;
    int angle = 0;

    auto response = process_user_input("invalid", *(tcp_client.get()), speed, angle);
    EXPECT_FALSE(response.has_value());
}
