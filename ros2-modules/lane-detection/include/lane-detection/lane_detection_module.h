#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <optional>

#include <opencv2/opencv.hpp>

struct LaneDetectionCfg{
    int cam_video_width{1640};
    int cam_video_height{1232};
    int cam_framerate{30};
};

struct LaneDetectionStatus {
    cv::Mat frame;
    std::string direction = "unknown";
    float steer_angle{0.0};
};

class LaneDetectionModule {
public:
    explicit LaneDetectionModule(LaneDetectionCfg cfg);
    ~LaneDetectionModule() = default;

    std::optional<LaneDetectionStatus> detect_lane(const cv::Mat& frame);
};
