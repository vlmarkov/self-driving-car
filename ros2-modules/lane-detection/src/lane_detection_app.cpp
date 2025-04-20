#include <lane-detection/lane_detection_module.h>

#include <opencv2/opencv.hpp>

struct CameraConfig {
    int capture_width{1280};
    int capture_height{720};
    int display_width{1280};
    int display_height{720};
    int framerate = 30 ;
    int flip_method = 0 ;
};

std::string gstreamer_pipeline (const CameraConfig& cfg)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)"
        + std::to_string(cfg.capture_width)
        + ", height=(int)"
        + std::to_string(cfg.capture_height)
        + ", framerate=(fraction)"
        + std::to_string(cfg.framerate)
        + "/1 ! nvvidconv flip-method="
        + std::to_string(cfg.flip_method)
        + " ! video/x-raw, width=(int)"
        + std::to_string(cfg.display_width)
        + ", height=(int)"
        + std::to_string(cfg.display_height)
        + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int test_camera_2()
{
    std::string pipeline = gstreamer_pipeline(CameraConfig());
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cout << "Failed to open camera." << std::endl;
        return (-1);
    }

    cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat img;

    std::cout << "Hit ESC to exit" << "\n" ;
    while(true)
    {
        if (!cap.read(img)) {
            std::cout<<"Capture read error"<<std::endl;
            break;
        }

        cv::imshow("CSI Camera", img);
        int keycode = cv::waitKey(10) & 0xff ;
        if (keycode == 27)
            break ;
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}

int test_camera_1()
{
    cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    const auto video_cam_dev = 0;
    if (!cap.open(video_cam_dev))
        return 0;

    for (;;) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty())
        break; // end of video stream

        cv::imshow("this is you, smile! :)", frame);
        if ((cv::waitKey(10) % 256) == 27)
            break; // waitKeypatch: check for 10ms: then stop capturing by pressing ESC=27
    }

    // the camera will be closed automatically upon exit
    cap.release();
    return 0;
}

int main(int argc, char* argv[]) {
    test_camera_1();
    test_camera_2();

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

    return 0;
}
