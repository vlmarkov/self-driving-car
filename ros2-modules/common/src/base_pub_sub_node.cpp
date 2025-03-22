#include <common/base_pub_sub_node.h>

namespace {

constexpr auto DEFAUL_QUEUE_SIZE = 100;

} // namespace

BasePubSubNode::BasePubSubNode(const PubSubCfg& cfg)
    : Node(cfg.name)
    , name_(cfg.name)
{
    publisher_ = this->create_publisher<interfaces::msg::MotionVector>(cfg.topic_publiser, DEFAUL_QUEUE_SIZE);

    auto pub_cb = [this]() -> void {
        interfaces::msg::MotionVector mv;
        {
            std::lock_guard<std::mutex> lock(publish_msg_mutex_);
            mv = publish_msg_;
        }

        RCLCPP_INFO(this->get_logger(), "I Publishing: auto-pilot: %d, acceleration: %f, steering: %f",
            mv.is_auto_pilot_on, mv.acceleration, mv.steering);

        this->publisher_->publish(mv);
    };
    timer_ = this->create_wall_timer(cfg.duration, pub_cb);

    auto sub_cb = [this](interfaces::msg::MotionVector::UniquePtr mv) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: auto-pilot: %d, acceleration: %f, steering: %f", 
            mv->is_auto_pilot_on, mv->acceleration, mv->steering);

        {
            std::lock_guard<std::mutex> lock(subscription_msg_mutex_);
            subscription_msg_.is_auto_pilot_on = mv->is_auto_pilot_on;
            subscription_msg_.acceleration = mv->acceleration;
            subscription_msg_.steering = mv->steering;
        }
    };
    subscription_ = this->create_subscription<interfaces::msg::MotionVector>(cfg.topic_subscription, DEFAUL_QUEUE_SIZE, sub_cb);
}

void BasePubSubNode::set_publish_msg(interfaces::msg::MotionVector msg)
{
    std::lock_guard<std::mutex> lock(publish_msg_mutex_);
    publish_msg_ = msg;
}

interfaces::msg::MotionVector BasePubSubNode::get_subscription_msg()
{
    std::lock_guard<std::mutex> lock(subscription_msg_mutex_);
    return subscription_msg_;
}

void BasePubSubNode::log(std::string log_msg) const {
    RCLCPP_INFO(this->get_logger(), log_msg.c_str());
}
