#include <network/tcp_client.h>
#include <network/socket.h>

#include <iostream>

int main(int argc, char * argv[])
{
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " [IP_ADDRESS] [PORT]" << std::endl;
        return -1;
    }

    auto address = std::string(argv[1]);
    auto port = ::atoi(argv[2]);

    TcpClient client(create_client_socket(), address, port);

    char read_char = ' ';
    Command cmd;
    while (1) {
        std::cin >> read_char;
        if (read_char == 'q') {
            std::cout << "Close " << argv[0] << std::endl;
            break;
        }

        if (read_char == 'e') {
            cmd = Command();
            cmd.is_auto_pilot_on = true;
        } else if (read_char == 'r') {
            cmd = Command();
        } else if (!cmd.is_auto_pilot_on && read_char == 'w') {
            cmd.acceleration += 10.0;
        } else if (!cmd.is_auto_pilot_on && read_char == 's') {
            cmd.acceleration -= 10.0;
        } else if (!cmd.is_auto_pilot_on && read_char == 'a') {
            cmd.steering -= 10.0;
        } else if (!cmd.is_auto_pilot_on && read_char == 'd') {
            cmd.steering += 10.0;
        }

        std::cout << "Auto-pilot   " << cmd.is_auto_pilot_on << std::endl;
        std::cout << "Acceleration " << cmd.acceleration << std::endl;
        std::cout << "Steering     " << cmd.steering << std::endl;

        client.send_command(cmd);
    }

    return 0;
}
