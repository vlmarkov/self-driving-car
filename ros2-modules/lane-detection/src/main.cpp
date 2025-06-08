#include <lane-detection/ros_module.h>

#include <thread>

#include <opencv2/opencv.hpp>

#ifdef ENABLE_RASPBERRY_BUILD

#include <lccv.hpp>
#include <libcamera_app.hpp>

#endif // ENABLE_RASPBERRY_BUILD

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<LaneDetection> ld) {
#ifdef ENABLE_RASPBERRY_BUILD
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cam.startVideo();

    cv::Mat frame = cv::Mat(0, 0, CV_8UC3);

    while(!stop_token.stop_requested()) {
        if (!cam.getVideoFrame(frame, 1000)) {
            std::cerr << "can't get image, timeout happend" << std::endl;
            continue;
        }

        ld->process_frame(frame);
    }

    cam.stopVideo();
#endif // ENABLE_RASPBERRY_BUILD
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto cfg = PubSubCfg{
        .name = LaneDetection::kName,
        .topic_publiser = std::string(LaneDetection::kName) + "Out",
        .topic_subscription = std::string(LaneDetection::kName) + "In",
        .duration = 500ms
    };
    auto pub_sub_node = std::make_shared<BasePubSubNode>(cfg);
    auto lane_detection = std::make_shared<LaneDetection>(pub_sub_node);

    std::jthread thread(run, lane_detection);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
