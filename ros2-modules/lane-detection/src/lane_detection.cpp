#include <lane-detection/lane_detection.h>

LaneDetection::LaneDetection(std::shared_ptr<IPubSubNode> pub_sub_node)
    : impl_(LaneDetectionCfg())
    , pub_sub_node_(pub_sub_node)
{
    // Nothing so far
}

void LaneDetection::process_frame(cv::Mat frame) {
    auto status = impl_.detect_lane(std::move(frame));

    auto mv = pub_sub_node_->get_subscription_msg();
    // TODO: do we need to correct acceleration field
    // mv.acceleration = 1.0;
    mv.steering = status.steer_angle;

    pub_sub_node_->set_publish_msg(std::move(mv));
}
