#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>

#include <opencv2/opencv.hpp>

class Lane {
public:
    Lane();
    Lane(int polyOrder, std::string color, int averagingCount);
    ~Lane();

    int getStableCenter(int coordinate);
    void setStartCoordinate(cv::Point point);
    cv::Point getStartCoordinate() const;
    void setStatus(bool flag);
    bool getStatus() const;
    void setPolyOrder(int value);
    int getPolyOrder() const;
    void setPolyCoeff(cv::Mat coeff);
    std::vector<float> getPolyCoeff() const;

private:
    int polyOrder;                // declare integer for order of line.
    std::string colour;           // set RGB values for lane colour.
    std::vector<float> polyCoeff; // Coefficients for equation
    cv::Point startCoordinates;   // Reference coordinates for line.

    // Average center to prevent jumps for entire run
    std::vector<int> averagingCenter;
    int averagingCount;
    int currentAveragingIndex;
    bool status;  // For status for program.
};
