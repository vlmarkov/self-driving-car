#include <lane-detection/ros_module.h>

LaneDetection::LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node, const LaneDetectionCfg& cfg)
    : impl_(cfg)
    , pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}

void LaneDetection::process_frame(const cv::Mat& frame) {
    if (frame.empty()) {
        return;
    }

    const auto result = impl_.detect_lane(frame);

#ifdef ENABLE_RASPBERRY_DEBUG_IMG
    if (result.has_value()) {
        cv::imshow("Lane Detection Status", result->frame);
    }
    cv::waitKey(50); // Wait some time to show image
#endif // ENABLE_RASPBERRY_DEBUG_IMG

    auto mv = pub_sub_node_->get_subscription_msg();
    if (!result.has_value()) {
        mv.acceleration = 0.0;
        mv.steering = 0.0;
    } else {
        mv.acceleration = 1.0;
        mv.steering = result->steer_angle;
    }
    // There is no point to send same data, and take cpu time
    // Raspberry Pi board does not have a lot of CPU power
    if (steer_angle_ != mv.steering) {
        steer_angle_ = mv.steering;
        pub_sub_node_->publish_msg(std::move(mv));
    }
}
