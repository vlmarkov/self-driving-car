#include <manual-control/ros_module.h>

#include <manual-control/network/tcp_server.h>
#include <manual-control/network/socket.h>

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<ManualControl> mc, std::string address, size_t port) {
    auto server = TcpServer(create_server_socket(), std::move(address), port);

    while(!stop_token.stop_requested()) {
        auto cmd = server.read_command();
        // TODO: support client connection lost
        mc->process_command(std::move(cmd));
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " [IP_ADDRESS] [PORT]" << std::endl;
        return -1;
    }

    auto address = std::string(argv[1]);
    auto port = ::atoi(argv[2]);

    auto cfg = PubSubCfg{
        .name = ManualControl::kName,
        .topic_publiser = std::string(ManualControl::kName) + "Out",
        .topic_subscription = std::string(ManualControl::kName) + "In",
        .duration = 500ms
    };
    auto pub_sub_node = std::make_shared<BasePubSubNode>(cfg);
    auto manual_control = std::make_shared<ManualControl>(pub_sub_node);

    std::jthread thread(run, manual_control, address, port);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
