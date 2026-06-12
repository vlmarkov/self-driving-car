#include <lane-detection/ros_module.h>

#include <thread>
#include <chrono>

#ifdef ENABLE_RASPBERRY_BUILD
#include <lccv.hpp>
#include <libcamera_app.hpp>
#endif // ENABLE_RASPBERRY_BUILD

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<LaneDetection> ld, const LaneDetectionCfg& cfg) {
#ifdef ENABLE_RASPBERRY_BUILD
    lccv::PiCamera cam;

    cam.options->video_width = cfg.cam_video_width;
    cam.options->video_height = cfg.cam_video_height;
    cam.options->framerate = cfg.cam_framerate;

    cam.startVideo();

    auto start_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
#endif // ENABLE_RASPBERRY_BUILD

    cv::Mat frame = cv::Mat(0, 0, CV_8UC3);

    while(!stop_token.stop_requested()) {
#ifdef ENABLE_RASPBERRY_BUILD
        if (!cam.getVideoFrame(frame, 1000)) {
            std::cerr << "can't get image, timeout happens!" << std::endl;
            continue;
        }
#endif // ENABLE_RASPBERRY_BUILD

        ld->process_frame(frame);

#ifdef ENABLE_RASPBERRY_BUILD
        frame_count++;

        const auto current_time = std::chrono::high_resolution_clock::now();
        const auto elapsed_time = current_time - start_time;

        if (elapsed_time.count() >= 1.0 /*sec*/) {
            double fps = frame_count / elapsed_time.count();
            std::cout << "FPS: " << fps << "\n";
            frame_count = 0; 
            start_time = std::chrono::high_resolution_clock::now();
        }
#endif // ENABLE_RASPBERRY_BUILD
    }

#ifdef ENABLE_RASPBERRY_BUILD
    cam.stopVideo();
    cv::destroyAllWindows();
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

    LaneDetectionCfg ld_cfg;
    pub_sub_node->declare_parameter<int>("cam_video_width", 1640);
    pub_sub_node->declare_parameter<int>("cam_video_height", 1232);
    pub_sub_node->declare_parameter<int>("cam_framerate", 30);

    ld_cfg.cam_video_width = pub_sub_node->get_parameter("cam_video_width").as_int();
    ld_cfg.cam_video_height = pub_sub_node->get_parameter("cam_video_height").as_int();
    ld_cfg.cam_framerate = pub_sub_node->get_parameter("cam_framerate").as_int();

    std::cout << "Starting " << LaneDetection::kName << std::endl;
    std::cout << "cam_video_width  " << ld_cfg.cam_video_width << std::endl;
    std::cout << "cam_video_height " << ld_cfg.cam_video_height << std::endl;
    std::cout << "cam_framerate    " << ld_cfg.cam_framerate << std::endl;

    auto lane_detection = std::make_shared<LaneDetection>(pub_sub_node, ld_cfg);

    std::jthread thread(run, lane_detection, ld_cfg);

    std::cout << "Started " << LaneDetection::kName << std::endl;

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    std::cout << "Stopped " << LaneDetection::kName << std::endl;

    return 0;
}
