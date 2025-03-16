#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "lane.h"

struct LaneDetectionCfg{
    cv::Scalar yellow_min{20, 100, 100};           // A yellow lane min threshold
    cv::Scalar yellow_max{30, 255, 255};           // A yellow lane max threshold
    int grayscale_min{200};                        // A white lane min threshold
    int grayscale_max{255};                        // A white lane max threshold
    cv::Point2f roi_upper_left_corner{300, 600};   // A camera/frame calibration point
    cv::Point2f roi_upper_right_corner{1000, 600}; // A camera/frame calibration point
};

struct LaneDetectionStatus {
    cv::Mat frame;
    std::string direction;
    double steer_angle;
};

class LaneDetectionModule {
public:
    LaneDetectionModule(LaneDetectionCfg cfg);
    ~LaneDetectionModule() = default;
    
    void undistortImage(const cv::Mat& src, cv::Mat& dst);
    void thresholdImageY(const cv::Mat& src, cv::Mat& dst);
    void thresholdImageW(const cv::Mat& src, cv::Mat& dst);
    void extractROI(const cv::Mat& src, cv::Mat& dst);
    void transformPerspective(const cv::Mat& src, cv::Mat& dst, cv::Mat& Tm, cv::Mat& invTm);
    void extractLanes(const cv::Mat& src, cv::Mat& dst, Lane& lane1, Lane& lane2, int curveFlag);
    void fitPoly(const std::vector<cv::Point>& src, cv::Mat& dst, int order);
    double getDriveHeading(Lane& lane1, Lane& lane2, std::string& direction);

    LaneDetectionStatus displayOutput(const cv::Mat& src, cv::Mat& src2, cv::Mat& dst, Lane& lane1, Lane& lane2, cv::Mat inv);
    LaneDetectionStatus detect_lane(cv::Mat frame);

    cv::Scalar getYellowMax();
    cv::Scalar getYellowMin();
    void setYellowMax(cv::Scalar value);
    void setYellowMin(cv::Scalar value);
    void setGrayScaleMax(int value);
    void setGrayScaleMin(int value);
    int getGrayScaleMin();
    int getGrayScaleMax();

private:
    cv::Scalar yellowMin_;               // Min possible RGB values of yellow
    cv::Scalar yellowMax_;               // Max possible RGB values of yellow
    int grayscaleMin_;                   // Min possible grayscale value for white
    int grayscaleMax_;                   // Max possible grayscale value for white
    cv::Point2f roi_upper_left_corner_;  // A camera/frame calibration point
    cv::Point2f roi_upper_right_corner_; // A camera/frame calibration point
};
