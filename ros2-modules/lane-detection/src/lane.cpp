#include <lane-detection/lane.h>

Lane::Lane() {
    polyOrder = 1;
    colour = "red";
    averagingCount = 10;
    currentAveragingIndex = 0;
    status = false;
}

Lane::Lane(int polyOrder, std::string color, int averagingCount) {
    this->polyOrder = polyOrder;
    this->colour = color;
    this->averagingCount = averagingCount;
    currentAveragingIndex = 0;
    status = false;
}

Lane::~Lane() {}

int Lane::getStableCenter(int coordinate) {
    if (currentAveragingIndex < averagingCount) {
        averagingCenter.push_back(coordinate);
        currentAveragingIndex++;
    } else {
        averagingCenter.erase(averagingCenter.begin());
        averagingCenter.push_back(coordinate);
    }

    int average = std::accumulate(averagingCenter.begin(), averagingCenter.end(), 0) / averagingCenter.size();
    return average;
}

void Lane::setStartCoordinate(cv::Point point) {
    startCoordinates = point;
}

void Lane::setStatus(bool flag) {
    status = flag;
}

void Lane::setPolyCoeff(cv::Mat laneCoeff) {
    polyCoeff.clear();
    if (polyCoeff.empty()) {
        polyCoeff.push_back(laneCoeff.at<float>(0, 0));
        polyCoeff.push_back(laneCoeff.at<float>(1, 0));
        polyCoeff.push_back(laneCoeff.at<float>(2, 0));
    } else {
        std::cerr << "Could not insert new elements!" << std::endl;
    }
}

cv::Point Lane::getStartCoordinate() const {
    return startCoordinates;
}

bool Lane::getStatus() const {
    return status;
}

std::vector<float> Lane::getPolyCoeff() const {
    return polyCoeff;
}

void Lane::setPolyOrder(int value) {
    polyOrder = value;
}

int Lane::getPolyOrder() const {
    return polyOrder;
}
