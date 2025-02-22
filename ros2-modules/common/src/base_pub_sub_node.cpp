#include <common/base_pub_sub_node.h>

BasePubSubNode::BasePubSubNode(const PubSubCfg& cfg)
    : Node(cfg.name)
    , name_(cfg.name)
{
    publisher_ = this->create_publisher<interfaces::msg::MotionVector>(cfg.topic_publiser, 100);

    auto pub_cb = [this]() -> void {
        auto mv = interfaces::msg::MotionVector();
        // TODO: !!!
        mv.acseleration = 1.0;
        mv.steering = -1.0;

        RCLCPP_INFO(this->get_logger(), "I Publishing: acseleration: %f, steering: %f", mv.acseleration, mv.steering);
        this->publisher_->publish(mv);
    };
    timer_ = this->create_wall_timer(cfg.duration, pub_cb);

    auto sub_cb = [this](interfaces::msg::MotionVector::UniquePtr mv) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: acseleration: %f, steering: %f", mv->acseleration, mv->steering);
    };
    subscription_ = this->create_subscription<interfaces::msg::MotionVector>(cfg.topic_subscription, 100, sub_cb);
}

void BasePubSubNode::set_publish_msg(interfaces::msg::MotionVector /*msg*/)
{
    // TODO: !!!
}

interfaces::msg::MotionVector BasePubSubNode::get_subscription_msg() const
{
    // TODO: !!!
    return interfaces::msg::MotionVector();
}
