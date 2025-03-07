#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "lane.h"

struct LaneDetectionInfo {
    cv::Mat frame;
    std::string direction;
    double steer_angle;
};

class LaneDetectionModule {
public:
    LaneDetectionModule();
    ~LaneDetectionModule();
    
    void undistortImage(const cv::Mat& src, cv::Mat& dst);
    void thresholdImageY(const cv::Mat& src, cv::Mat& dst);
    void thresholdImageW(const cv::Mat& src, cv::Mat& dst);
    void extractROI(const cv::Mat& src, cv::Mat& dst);
    void transformPerspective(const cv::Mat& src, cv::Mat& dst, cv::Mat& Tm, cv::Mat& invTm);
    void extractLanes(const cv::Mat& src, cv::Mat& dst, Lane& lane1, Lane& lane2, int curveFlag);
    void fitPoly(const std::vector<cv::Point>& src, cv::Mat& dst, int order);
    double getDriveHeading(Lane& lane1, Lane& lane2, std::string& direction);

    LaneDetectionInfo displayOutput(const cv::Mat& src, cv::Mat& src2, cv::Mat& dst, Lane& lane1, Lane& lane2, cv::Mat inv);
    LaneDetectionInfo detectLane(cv::Mat frame);

    cv::Scalar getYellowMax();
    cv::Scalar getYellowMin();
    void setYellowMax(cv::Scalar value);
    void setYellowMin(cv::Scalar value);
    void setGrayScaleMax(int value);
    void setGrayScaleMin(int value);
    int getGrayScaleMin();
    int getGrayScaleMax();

private:
    cv::Scalar yellowMin_;  // max possible RGB values of yellow
    cv::Scalar yellowMax_;  // min possible RGB values of yellow
    int grayscaleMin_;      // min possible grayscale value for white in our video
    int grayscaleMax_;      // max possible grayscale value for white in our video
};
