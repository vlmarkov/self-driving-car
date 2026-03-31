#include <remote-control/remote-control/commands.h>

namespace {

std::vector<std::string> split_by_char(const std::string& str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    // Use getline to extract tokens separated by a specific character
    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

} // namespace

Request::Request(std::string str) {
    if (nlohmann::json::accept(str)) {
        json_ = nlohmann::json::parse(str);

        // TODO: j_.contains("request")
        return;
    }

    auto ss = split_by_char(str, DELIMITER);

    std::string command = ss.at(0);
    std::string args = ss.size() > 1 ? ss.at(1) : "";

    json_ = nlohmann::json{
        {
            "request", {
                {"command", command},
                {"args", args},
            }
        }
    };
}

Request::Request(const int speed, const int angle) {
    json_ = nlohmann::json{
        {
            "request", {
                {"command", REMOTE_CONTROL},
                {"args", {
                    {SPEED, speed},
                    {ANGLE, angle}
                }},
            }
        }
    };
}

std::string Request::get_command() const {
    return json_.at("request").at("command");
}

std::string Request::get_args() const {
    return json_.at("request").at("args").dump();
}

std::string Request::serialize() const {
    return json_.dump();
}

Response::Response() {
    json_ = nlohmann::json{
        {
            "response", {
                {"code", ""},
                {"data", ""}
            }
        }
    };
}

Response::Response(std::string json_str) {
    if (!nlohmann::json::accept(json_str)) {
        throw std::runtime_error("invalid string to build json response object");
    }

    json_ = nlohmann::json::parse(json_str);
    if (!json_.contains("response")) {
        throw std::runtime_error("invalid json response object, miss response field");
    }
}

void Response::set_code(std::string code) {
    json_["response"]["code"]= code;
}

void Response::set_data(std::string data) {
    json_["response"]["data"]= data;
}

std::string Response::get_code() const {
    return json_.at("response").at("code");
}

std::string Response::get_data() const {
    return json_.at("response").at("data");
}

std::string Response::serialize() const {
    return json_.dump();
}
