#include <remote-control/remote_control_server.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

class MockPubSubNode : public IPubSubNode {
public:
    MOCK_METHOD(void, log, (std::string log_msg), (const, final));
    MOCK_METHOD(void, publish_msg, (interfaces::msg::MotionVector msg), (final));
    MOCK_METHOD(interfaces::msg::MotionVector, get_subscription_msg, (), (final));
};

class MockSysExecutor : public ISysExecutor {
public:
    MOCK_METHOD(Response, execute, (const SystemCommand& cmd), (final));
};

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

class RemoteControlServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto mock_server_socket = std::make_unique<MockServerSocket>();
        auto mock_client_socket = std::make_unique<MockClientSocket>();

        mock_client_socket_ptr = mock_client_socket.get();

        EXPECT_CALL(*(mock_client_socket.get()), is_connected).Times(1).WillOnce(
        ::testing::Return(true)
        );
        EXPECT_CALL(*(mock_server_socket.get()), accept_client).Times(1).WillOnce(
            ::testing::Return(std::move(mock_client_socket))
        );

        tcp_server = std::make_shared<TcpServer>(
            std::move(mock_server_socket),
            "localhost",
            8080
        );
    }

    MockClientSocket *mock_client_socket_ptr = nullptr;

    std::shared_ptr<SystemState> sys_state = std::make_shared<SystemState>();
    std::shared_ptr<MockPubSubNode> ros_node = std::make_shared<MockPubSubNode>();
    std::shared_ptr<MockSysExecutor> sys_exec = std::make_shared<MockSysExecutor>();
    std::shared_ptr<RosExecutor> ros_exec = std::make_shared<RosExecutor>();
    std::shared_ptr<TcpServer> tcp_server;
};

} // namespace

TEST_F(RemoteControlServerTest, WithInvalidRequestExpectErrorResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request("invalid").serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        response.set_code("error");
        response.set_data("invalid request");

        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithBuildSwExpectValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(BUILD_SW).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithUpdateSwValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(UPDATE_SW).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithStartSwValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(START_SW).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithStopSwValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(STOP_SW).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithRestartSwValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(RESTART_SW).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        EXPECT_EQ(s, response.serialize());
        return 1;
    });

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithStartAutoPilotValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(START_AUTO_PILOT).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        response.set_code("ok");
        EXPECT_EQ(s, response.serialize());
        return 1;
    });
    EXPECT_CALL(*(ros_node.get()), publish_msg).Times(1).WillOnce([](interfaces::msg::MotionVector actual) {
        interfaces::msg::MotionVector expected;
        expected.is_auto_pilot_on = true;
        expected.acceleration = 0;
        expected.steering = 0;

        EXPECT_EQ(actual, expected);
        return 1;
    });

    sys_state->is_sw_running = true;
    sys_state->is_auto_pilot_on = false;

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithStopAutoPilotValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(STOP_AUTO_PILOT).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        response.set_code("ok");
        EXPECT_EQ(s, response.serialize());
        return 1;
    });
    EXPECT_CALL(*(ros_node.get()), publish_msg).Times(1).WillOnce([](interfaces::msg::MotionVector actual) {
        interfaces::msg::MotionVector expected;
        expected.is_auto_pilot_on = false;
        expected.acceleration = 0;
        expected.steering = 0;

        EXPECT_EQ(actual, expected);
        return 1;
    });

    sys_state->is_sw_running = true;
    sys_state->is_auto_pilot_on = true;

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}

TEST_F(RemoteControlServerTest, WithRemoteControlValidResponse) {
    EXPECT_CALL(*mock_client_socket_ptr, read_data).Times(1).WillOnce(
        [](std::string& s) { s = Request(10, 10).serialize(); return 1; }
    );
    EXPECT_CALL(*mock_client_socket_ptr, send_data).Times(1).WillOnce([](const std::string& s) {
        Response response;
        response.set_code("ok");
        EXPECT_EQ(s, response.serialize());
        return 1;
    });
    EXPECT_CALL(*(ros_node.get()), publish_msg).Times(1).WillOnce([](interfaces::msg::MotionVector actual) {
        interfaces::msg::MotionVector expected;
        expected.is_auto_pilot_on = false;
        expected.acceleration = 10;
        expected.steering = 10;

        EXPECT_EQ(actual, expected);
        return 1;
    });

    sys_state->is_sw_running = true;
    sys_state->is_auto_pilot_on = false;

    process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
}