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
    void displayOutput(const cv::Mat& src, cv::Mat& src2, cv::Mat& dst, Lane& lane1, Lane& lane2, cv::Mat inv);

    bool detectLane(std::string videoName);
    cv::Scalar getYellowMax();
    cv::Scalar getYellowMin();
    void setYellowMax(cv::Scalar value);
    void setYellowMin(cv::Scalar value);
    void setGrayScaleMax(int value);
    void setGrayScaleMin(int value);
    int getGrayScaleMin();
    int getGrayScaleMax();

private:
    cv::Scalar yellowMin;  // max possible RGB values of yellow
    cv::Scalar yellowMax;  // min possible RGB values of yellow
    int grayscaleMin;      // min possible grayscale value for white in our video
    int grayscaleMax;      // max possible grayscale value for white in our video
    std::string videoName; // specify video name
};
