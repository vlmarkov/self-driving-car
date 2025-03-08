#pragma once

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motion_vector.hpp"

struct PubSubCfg {
    const std::string name;
    const std::string topic_publiser;
    const std::string topic_subscription;
    std::chrono::milliseconds duration;
};

class IPubSubNode
{
public:
    virtual ~IPubSubNode() = default;

    virtual void set_publish_msg(interfaces::msg::MotionVector msg) = 0;
    virtual interfaces::msg::MotionVector get_subscription_msg() = 0;
};

class BasePubSubNode : public IPubSubNode, public rclcpp::Node
{
public:
    BasePubSubNode(const PubSubCfg& cfg);
    ~BasePubSubNode() = default;

    void set_publish_msg(interfaces::msg::MotionVector msg) final;
    interfaces::msg::MotionVector get_subscription_msg() final;

private:
    const std::string name_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<interfaces::msg::MotionVector>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::MotionVector>::SharedPtr subscription_;

    interfaces::msg::MotionVector publish_msg_;
    interfaces::msg::MotionVector subscription_msg_;

    std::mutex publish_msg_mutex_;
    std::mutex subscription_msg_mutex_;
};
