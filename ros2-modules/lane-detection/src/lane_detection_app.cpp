#include <lane-detection/lane_detection_module.h>

#ifdef ENABLE_RASPBERY_BUILD

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

int test_raspbery_camera() {
    uint32_t num_cams = LibcameraApp::GetNumberCameras();
    std::cout << "Found " << num_cams << " cameras." << std::endl;

    uint32_t height = 720;
    uint32_t width = 1280;

    std::cout << "Sample program for LCCV video capture" << std::endl;
    std::cout << "Press ESC to stop." << std::endl;

    cv::Mat image = cv::Mat(height, width, CV_8UC3);
    lccv::PiCamera cam;
    cam.options->video_width=width;
    cam.options->video_height=height;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cam.startVideo();

    int ch = 0;
    LaneDetectionModule lm({});

    while (ch != 27) {
        if (!cam.getVideoFrame(image,1000)) {
            std::cout<<"Timeout error"<<std::endl;
        } else {
            auto frame = image;
            auto status = lm.detect_lane(std::move(frame));
            cv::imshow("Lane Detection Status", status.frame);
            std::cout << status.direction << std::endl;
            std::cout << status.steer_angle << std::endl;
        }

        ch = cv::waitKey(5);
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}

#endif // ENABLE_RASPBERY_BUILD

int test_static_image(int argc, char* argv[]) {
    auto frame = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        return -1;
    }

    LaneDetectionModule lm({});
    auto status = lm.detect_lane(std::move(frame));

    cv::imshow("Lane Detection Status", status.frame);
    std::cout << status.direction << std::endl;
    std::cout << status.steer_angle << std::endl;

    if (cv::waitKey(3000) >= 0) {
        return 0;
    }
}

int main(int argc, char* argv[]) {
#ifdef ENABLE_RASPBERY_BUILD
    test_raspbery_camera();
#endif // ENABLE_RASPBERY_BUILD

    test_static_image(argc, argv);

    return 0;
}
