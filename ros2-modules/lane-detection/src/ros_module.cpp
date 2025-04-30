#include <lane-detection/ros_module.h>

LaneDetection::LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node)
    : impl_(LaneDetectionCfg())
    , pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}

void LaneDetection::process_frame(cv::Mat frame) {
    auto status = impl_.detect_lane(std::move(frame));

#ifdef ENABLE_RASPBERRY_DEBUG_IMG
    cv::imshow("Lane Detection Status", status.frame);
    std::cout << status.direction << std::endl;
    std::cout << status.steer_angle << std::endl;
#endif // ENABLE_RASPBERRY_DEBUG_IMG

    auto mv = pub_sub_node_->get_subscription_msg();
    if (status.frame.empty()) {
        mv.acceleration = 0.0;
        mv.steering = 0.0;
    } else {
        // TODO: do we need to correct acceleration field
        mv.acceleration = 1.0;
        mv.steering = status.steer_angle;        
    }

    pub_sub_node_->publish_msg(std::move(mv));
}
