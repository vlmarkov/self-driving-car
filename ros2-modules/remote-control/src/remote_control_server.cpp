#include <remote-control/remote_control_server.h>
#include <remote-control/network/socket.h>

namespace {

SystemCommand create_build_sw_cmd() {
    // TODO: !
    return "echo colcon build";
}

SystemCommand create_update_sw_cmd() {
    // TODO: !
    return "echo git pull";
}

SystemCommand create_stop_sw_cmd() {
    // TODO: !
    return "echo kill all";
}

SystemCommand create_start_sw_cmd(const std::string& args) {
    // TODO: !
    return "echo ros2 run " + args;
}

SystemCommand create_restart_sw_cmd(const std::string& args) {
    // TODO: !
    return "echo kill all && ros2 run " + args;
}

} // namespace

Response SystemExecutor::execute(const SystemCommand& cmd) {
    Response response;
    response.set_code("ok");

    if (std::system(cmd.c_str()) != 0) {
        response.set_code("error");
    }

    return response;
}

Response RosExecutor::execute(const Request& request, std::shared_ptr<IPubSubNode> ros_node) {
    Response response;
    response.set_code("ok");

    auto mv = ros_node->get_subscription_msg();

    if (request.get_command() == START_AUTO_PILOT) {
        mv.is_auto_pilot_on = true;
        mv.acceleration = 0;
        mv.steering = 0;
    } else if (request.get_command() == STOP_AUTO_PILOT) {
        mv.is_auto_pilot_on = false;
        mv.acceleration = 0;
        mv.steering = 0;
    } else {
        const auto json = nlohmann::json::parse(request.get_args());

        mv.is_auto_pilot_on = false;
        mv.acceleration = json[SPEED].get<float>();
        mv.steering = json[ANGLE].get<float>();
    }

    ros_node->publish_msg(std::move(mv));
    // TODO: error support
    return response;
}

void process_remote_request(
    std::shared_ptr<SystemState> sys_state,
    std::shared_ptr<TcpServer> tcp_server,
    std::shared_ptr<IPubSubNode> ros_node,
    std::shared_ptr<ISysExecutor> sys_exec,
    std::shared_ptr<IRosExecutor> ros_exec
)
{
    std::string buffer;
    buffer.resize(MAX_DATA_SIZE);

    if (tcp_server->read_request(buffer) < 1) {
        return;
    }

    Request request(buffer);

    Response response;
    response.set_code("error");
    response.set_data("invalid request");

    if (request.get_command() == BUILD_SW) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before build-sw, you must stop auto-pilot");
        } else if (sys_state->is_sw_running == true) {
            response.set_data("before build-sw, you must stop software");
        } else {
            response = sys_exec->execute(create_build_sw_cmd());
        }
    } else if (request.get_command() == UPDATE_SW) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before update-sw, you must stop auto-pilot");
        } else if (sys_state->is_sw_running == true) {
            response.set_data("before update-sw, you must stop software");
        } else {
            response = sys_exec->execute(create_update_sw_cmd());
        }
    } else if (request.get_command() == START_SW) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before start-sw, you must stop auto-pilot");
        } else if (sys_state->is_sw_running == true) {
            response.set_data("software already running");
        } else {
            response = sys_exec->execute(create_start_sw_cmd(request.get_args()));
            if (response.get_code() == "ok") {
                sys_state->is_sw_running = true;
            }
        }
    } else if (request.get_command() == STOP_SW) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before start-sw, you must stop auto-pilot");
        } else {
            response = sys_exec->execute(create_stop_sw_cmd());
            if (response.get_code() == "ok") {
                sys_state->is_sw_running = false;
            }
        }
    } else if (request.get_command() == RESTART_SW) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before restart-sw, you must stop auto-pilot");
        } else {
            response = sys_exec->execute(create_restart_sw_cmd(request.get_args()));
            if (response.get_code() == "ok") {
                sys_state->is_sw_running = true;
            }
        }
    } else if (request.get_command() == START_AUTO_PILOT) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("auto-pilot already running");
        } else if (sys_state->is_sw_running == false) {
            response.set_data("before start auto-pilot, you must start software");
        } else {
            response = ros_exec->execute(request, ros_node);
            if (response.get_code() == "ok") {
                sys_state->is_auto_pilot_on = true;
            }
        }
    } else if (request.get_command() == STOP_AUTO_PILOT) {
        if (sys_state->is_sw_running == false) {
            response.set_data("software is not running");
        } else {
            response = ros_exec->execute(request, ros_node);
            if (response.get_code() == "ok") {
                sys_state->is_auto_pilot_on = false;
            }
        }
    } else if (request.get_command() == REMOTE_CONTROL) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before remote-control, you must stop auto-pilot");
        } else if (sys_state->is_sw_running == false) {
            response.set_data("before remote-control, you must start software");
        } else {
            response = ros_exec->execute(request, ros_node);
        }
    } else if (request.get_command() == TEST_CMD) {
        if (sys_state->is_auto_pilot_on == true) {
            response.set_data("before play-remote-control, you must stop auto-pilot");
        } else if (sys_state->is_sw_running == false) {
            response.set_data("before play-remote-control, you must start software");
        } else {
            // TODO: add support of test scenario commands
            response = ros_exec->execute(request, ros_node);
        }
    }

    if (tcp_server->send_response(response.serialize()) < 1) {
        return;
    }
}
