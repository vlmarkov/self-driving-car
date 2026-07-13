#include <common/base_pub_sub_node.h>

BasePubSubNode::BasePubSubNode(const PubSubCfg& cfg)
    : Node(cfg.name)
    , name_(cfg.name)
{
    publisher_ = this->create_publisher<interfaces::msg::MotionVector>(cfg.topic_publiser, DEFAULT_QUEUE_SIZE);
    
    subscription_msg_.is_auto_pilot_on = false;
    subscription_msg_.acceleration = 0.0;
    subscription_msg_.steering = 0.0;

    auto sub_cb = [this](interfaces::msg::MotionVector::UniquePtr mv) -> void {
        if (subscription_msg_.is_auto_pilot_on == mv->is_auto_pilot_on &&
            subscription_msg_.acceleration == mv->acceleration &&
            subscription_msg_.steering == mv->steering)
        {
          return;
        }
      
        RCLCPP_INFO(this->get_logger(), "receive: auto-pilot: %d, acceleration: %f, steering: %f", 
            mv->is_auto_pilot_on, mv->acceleration, mv->steering);

        {
            std::lock_guard<std::mutex> lock(subscription_msg_mutex_);
            subscription_msg_.is_auto_pilot_on = mv->is_auto_pilot_on;
            subscription_msg_.acceleration = mv->acceleration;
            subscription_msg_.steering = mv->steering;
        }
    };
    subscription_ = this->create_subscription<interfaces::msg::MotionVector>(cfg.topic_subscription, DEFAULT_QUEUE_SIZE, sub_cb);
}

void BasePubSubNode::publish_msg(interfaces::msg::MotionVector msg)
{
    RCLCPP_INFO(this->get_logger(), "publish: auto-pilot: %d, acceleration: %f, steering: %f",
        msg.is_auto_pilot_on, msg.acceleration, msg.steering);

    this->publisher_->publish(std::move(msg));
}

interfaces::msg::MotionVector BasePubSubNode::get_subscription_msg()
{
    std::lock_guard<std::mutex> _(subscription_msg_mutex_);
    return subscription_msg_;
}

void BasePubSubNode::log(std::string log_msg) const {
    RCLCPP_INFO(this->get_logger(), log_msg.c_str());
}
