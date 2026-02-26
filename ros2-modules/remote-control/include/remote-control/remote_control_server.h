#pragma once

#include <common/base_pub_sub_node.h>
#include <remote-control/network/tcp_server.h>
#include <remote-control/remote_control_commands.h>

class ISysExecutor
{
public:
    virtual ~ISysExecutor() = default;

    virtual Response execute(const SystemCommand& cmd) = 0;

protected:
    ISysExecutor() = default;
};

class IRosExecutor
{
public:
    virtual ~IRosExecutor() = default;

    virtual Response execute(const Request& request, std::shared_ptr<IPubSubNode> ros_node) = 0;

protected:
    IRosExecutor() = default;
};

class SystemExecutor : public ISysExecutor {
public:
    SystemExecutor() = default;
    ~SystemExecutor() = default;

    Response execute(const SystemCommand& cmd) final;
};

class RosExecutor : public IRosExecutor {
public:
    RosExecutor() = default;
    ~RosExecutor() = default;

    Response execute(const Request& request, std::shared_ptr<IPubSubNode> ros_node) final;
};

void process_remote_request(
    std::shared_ptr<SystemState> sys_state,
    std::shared_ptr<TcpServer> tcp_server,
    std::shared_ptr<IPubSubNode> ros_node,
    std::shared_ptr<ISysExecutor> sys_exec,
    std::shared_ptr<IRosExecutor> ros_exec
);
