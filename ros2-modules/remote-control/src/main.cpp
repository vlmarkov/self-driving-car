#include <remote-control/remote-control/server.h>
#include <remote-control/network/socket.h>

using namespace std::chrono_literals;

namespace {

constexpr auto KNAME = "RemoteControl";

void run_server(std::stop_token stop_token,
                std::shared_ptr<BasePubSubNode> ros_node,
                const std::string& address,
                const size_t port)
{
    auto sys_state = std::make_shared<SystemState>();
    auto tcp_server = std::make_shared<TcpServer>(create_server_socket(), address, port);
    auto sys_exec = std::make_shared<SystemExecutor>();
    auto ros_exec = std::make_shared<RosExecutor>();

    while(!stop_token.stop_requested()) {
        try {
            process_remote_request(sys_state, tcp_server, ros_node, sys_exec, ros_exec);
        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
            sys_state = std::make_shared<SystemState>();
            tcp_server = std::make_shared<TcpServer>(create_server_socket(), address, port);
            sys_exec = std::make_shared<SystemExecutor>();
            ros_exec = std::make_shared<RosExecutor>();
        }
    }
}

void help(const char* argv) {
    std::cerr << "Usage: " << argv << " [IP_ADDRESS] [PORT]" << std::endl;
}

} // namespace

int main(int argc, char * argv[])
{
    if (argc != 3) {
        help(argv[0]);
        return -1;
    }

    const auto address = std::string(argv[1]);
    const auto port = ::atoi(argv[2]);

    rclcpp::init(argc, argv);

    auto cfg = PubSubCfg{
        .name = KNAME,
        .topic_publiser = std::string(KNAME) + "Out",
        .topic_subscription = std::string(KNAME) + "In",
        .duration = 500ms
    };

    auto ros_node = std::make_shared<BasePubSubNode>(cfg);
    std::jthread thread(run_server, ros_node, address, port);

    rclcpp::spin(ros_node);
    rclcpp::shutdown();

    return 0;
}
