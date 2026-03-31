#include <remote-control/network/socket.h>
#include <remote-control/remote-control/client.h>

#include <string>
#include <iostream>

namespace {

void help(const char* argv) {
    std::cerr << "Usage: " << argv << " [IP_ADDRESS] [PORT]" << std::endl;
}

} // namespace

int main(int argc, const char* argv[])
{
    try {
        if (argc != 3) {
            help(argv[0]);
            return -1;
        }

        TcpClient client(
            create_client_socket(),
            std::string(argv[1]),
            static_cast<uint16_t>(::atoi(argv[2]))
        );

        int speed = 0;
        int angle = 0;

        while (1) {
            std::cout << "input command: " << std::endl;
            std::string user_input;
            std::getline(std::cin, user_input);
            std::cout << "input command is: " << user_input << std::endl;
            if (user_input == EXIT) {
                break;
            }

            auto reponse = process_user_input(user_input, client, speed, angle);
            if (reponse.has_value()) {
                continue;
            }

            std::cerr << "Unknown user input: " << user_input << std::endl;
        }
    } catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
