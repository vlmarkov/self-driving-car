#include <lane-detection/lane_detection_module.h>

LaneDetectionModule::LaneDetectionModule(LaneDetectionCfg cfg)
    : yellowMin_(cfg.yellow_min)
    , yellowMax_(cfg.yellow_max)
    , grayscaleMin_(cfg.grayscale_min)
    , grayscaleMax_(cfg.grayscale_max)
    , roi_upper_left_corner_(cfg.roi_upper_left_corner)
    , roi_upper_right_corner_(cfg.roi_upper_right_corner)
{
    // Nothing so far
}

void LaneDetectionModule::undistortImage(const cv::Mat& src, cv::Mat& dst) {
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1154.22732, 0.0, 671.627794, 0.0, 1148.18221, 386.046312, 0.0, 0.0, 1.0);
    std::vector<double> distortionCoeff { -.242565104, -0.0477893070, -0.00131388084, -0.0000879107779, 0.0220573263 };
    cv::undistort(src, dst, cameraMatrix, distortionCoeff);
}

void LaneDetectionModule::thresholdImageY(const cv::Mat& src, cv::Mat& dst) {
    // Image to HLS type image for decreasing effect of light intensity.
    cv::cvtColor(src, dst, cv::COLOR_BGR2HLS);

    // use white thresholding values to detect only road lanes
    cv::inRange(dst, yellowMin_, yellowMax_, dst);
}

void LaneDetectionModule::thresholdImageW(const cv::Mat& src, cv::Mat& dst) {
    // Convert to grayscale image
    cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);

    // use white thresholding values to detect only road lanes
    cv::inRange(dst, grayscaleMin_, grayscaleMax_, dst);
}

void LaneDetectionModule::extractROI(const cv::Mat& src, cv::Mat& dst) {
    int width = src.cols;  // width of recieved frame.
    int height = src.rows;  // height of recieved frame.

    //  mask matrix initialized as zero Matrix of Height and Width of frame matrix.
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
    // Make corners points for the mask.
    // Calibrateted for black and white sample
    cv::Point pts[4] = {
        cv::Point2f(width, height), // b r c
        cv::Point2f(0, height),     // b l c
        roi_upper_left_corner_,     // u l c
        roi_upper_right_corner_     // u r c
    };

    // Create a polygon
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));

    // And the source thresholded image and mask
    bitwise_and(src, mask, dst);
}

void LaneDetectionModule::transformPerspective(const cv::Mat& src, cv::Mat& dst, cv::Mat& Tm, cv::Mat& invTm) {
    const auto w = src.cols;
    const auto h = src.rows;

    dst = src.clone();

    // Make corners for the transform
    // br, bl, tl, tr -> order
    const cv::Point2f start[4] = {
        cv::Point2f(w, h),      // b r c
        cv::Point2f(0, h),      // b l c
        roi_upper_left_corner_, // u l c 
        roi_upper_right_corner_ // u r c
    };

    // Must be the same as original
    const cv::Point2f end[4] = {
        cv::Point2f(w, h),
        cv::Point2f(0, h),
        cv::Point2f(0, 0),
        cv::Point2f(w, 0)
    };

    Tm = getPerspectiveTransform(start, end);
    invTm = getPerspectiveTransform(end, start);

    cv::warpPerspective(src, dst, Tm, cv::Size(w, h));
}

void LaneDetectionModule::extractLanes(const cv::Mat& src, cv::Mat& colorLane, Lane& lane1, Lane& lane2, int curveFlag) {
    const int w = src.cols;
    const int h = src.rows;

    // Height to start the sliding windows
    const int padding = 10;
    const int bottomHeight = h - padding;
    const int bottomWidth = w - padding;

    // Convert the gray src image to bgr for plotting colors(3-channel)
    cv::cvtColor(src, colorLane, cv::COLOR_GRAY2BGR);

    // Create a mask for bottom half to get the histograms
    cv::Rect roi(0, h / 2, w, h / 2);
    cv::Mat croppedIm = src(roi);

    // Reduce the input matrix to a single row
    std::vector<double> histogram;
    cv::reduce(croppedIm, histogram, 0, cv::REDUCE_SUM);

    // Split the two vectors for left and right lane
    std::size_t const halfSize = histogram.size() / 2;
    std::vector<double> leftHist(histogram.begin(), histogram.begin() + halfSize);
    std::vector<double> rightHist(histogram.begin() + halfSize, histogram.end());

    // Get the max element in left half
    int maxLeftIndex = std::max_element(leftHist.begin(), leftHist.end()) - leftHist.begin();

    // Get the max element in left half
    int maxRightIndex = std::max_element(rightHist.begin(), rightHist.end()) - rightHist.begin();

    maxRightIndex = maxRightIndex + halfSize;

    // Normalize the right lane as there are breaks in the
    // lane-Optional for stability
    maxRightIndex = lane2.getStableCenter(maxRightIndex);

    // Declare vectors of Point for storing lane points
    std::vector<cv::Point> leftLane;
    std::vector<cv::Point> rightLane;

    // Sliding Window approach
    const int windowCount = 9;
    const int windowHeight = (h - padding) / windowCount;
    const int windowWidth = windowHeight * 2;

    // Left Lane ***********************************************************
    int currentHeight = bottomHeight;
    // Assign start coordinates
    lane1.setStartCoordinate(cv::Point(maxLeftIndex, currentHeight));
    for (int i = 0; i < windowCount; i++) {
        // Get the top left and bottom right point to make a rectangle
        int tlX = maxLeftIndex - windowWidth / 2;
        int tlY = currentHeight - windowHeight;
        int brX = maxLeftIndex + windowWidth / 2;
        int brY = currentHeight;

        // boundary checks
        tlX = (tlX < 0) ? padding : tlX;
        tlY = (tlY < 0) ? padding : tlY;

        brX = (brX > w) ? bottomWidth : brX;
        brY = (brY > h) ? bottomHeight : brY;

        cv::Point tl(tlX, tlY);
        cv::Point br(brX, brY);

        cv::rectangle(colorLane, tl, br, cv::Scalar(0, 255, 204), 3);

        // Create a temporary vector to store the x's for next box
        std::vector<int> nextX;
        // Get the location of the white pixels to store in vector
        int pointFlag = 0;
        for (int j = tlX; j <= brX; j++) {
            for (int k = tlY; k <= brY; k++) {
                if (src.at<uchar>(k, j) > 0) {
                    colorLane.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(0, 0, 255);
                    if (pointFlag % 10 == 0) {
                        leftLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
                    }
                    pointFlag++;
                    nextX.push_back(j);
                }
            }
        }

        // Get the center of the next bounding box
        if (!nextX.empty()) {
            maxLeftIndex = std::accumulate(nextX.begin(), nextX.end(), 0) / nextX.size();
        }
        currentHeight = currentHeight - windowHeight;
    }

    // Right Lane ****************************************************
    currentHeight = bottomHeight;
    // Assign start coordinates
    lane2.setStartCoordinate(cv::Point(maxRightIndex, currentHeight));
    for (int i = 0; i < windowCount; i++) {
        // Get the top left and bottom right point to make a rectangle
        int tlX = maxRightIndex - windowWidth / 2;
        int tlY = currentHeight - windowHeight;
        int brX = maxRightIndex + windowWidth / 2;
        int brY = currentHeight;

        // boundary checks
        tlX = (tlX < 0) ? padding : tlX;
        tlY = (tlY < 0) ? padding : tlY;

        brX = (brX > w) ? bottomWidth : brX;
        brY = (brY > h) ? bottomHeight : brY;

        cv::Point tl(tlX, tlY);
        cv::Point br(brX, brY);

        cv::rectangle(colorLane, tl, br, cv::Scalar(0, 255, 204), 3);

        // Create a temporary vector to store the x's for next box
        std::vector<int> nextX;

        // Get the location of the white pixels to store in vector
        int pointFlag = 0;
        for (int j = tlX; j <= brX; j++) {
            for (int k = tlY; k <= brY; k++) {
                if (src.at<uchar>(k, j) > 0) {
                    colorLane.at<cv::Vec3b>(cv::Point(j, k)) = cv::Vec3b(0, 255, 0);
                    if (pointFlag % 10 == 0) {
                        rightLane.push_back(cv::Point(k, j));  // Push as row(y), col(x)
                    }
                    pointFlag++;
                    nextX.push_back(j);
                }
            }
        }

        // Get the center of the next bounding box
        if (!nextX.empty()) {
            maxRightIndex = std::accumulate(nextX.begin(), nextX.end(), 0) / nextX.size();
        }

        currentHeight = currentHeight - windowHeight;
    }

    if (!leftLane.empty()) {
        // Mat objects for lane parameters
        cv::Mat leftLaneParams = cv::Mat::zeros(curveFlag + 1, 1, CV_64F);
        // Call the fitpoly function
        fitPoly(leftLane, leftLaneParams, curveFlag);
        // Assign to the lane object and set params
        lane1.setPolyCoeff(leftLaneParams);
        // Assign status
        lane1.setStatus(true);
    }

    if (!rightLane.empty()) {
        // Mat objects for lane parameters
        cv::Mat rightLaneParams = cv::Mat::zeros(curveFlag + 1, 1, CV_64F);
        // Call the fitpoly function
        fitPoly(rightLane, rightLaneParams, curveFlag);
        // Assign to the lane object and set params
        lane2.setPolyCoeff(rightLaneParams);
        lane2.setStatus(true);
    }
}

void LaneDetectionModule::fitPoly(const std::vector<cv::Point>& src, cv::Mat& dst, int order) {
    if (src.empty())
        return;

    cv::Mat x = cv::Mat(src.size(), 1, CV_32F);
    cv::Mat y = cv::Mat(src.size(), 1, CV_32F);

  // Add all the points to the mat
  for (size_t i = 0; i < src.size(); i++) {
    x.at<float>(i, 0) = static_cast<float>(src[i].x);
    y.at<float>(i, 0) = static_cast<float>(src[i].y);
  }

    int npoints = x.checkVector(1);
    int nypoints = y.checkVector(1);
    CV_Assert(npoints == nypoints && npoints >= order + 1);
    cv::Mat_<double> srcX(x), srcY(y);
    cv::Mat_<double> A = cv::Mat_<double>::ones(npoints, order + 1);
    // build A matrix
    for (int y = 0; y < npoints; ++y) {
        for (int x = 1; x < A.cols; ++x)
        A.at<double>(y, x) = srcX.at<double>(y) * A.at<double>(y, x - 1);
    }
    cv::Mat w;
    cv::solve(A, srcY, w, cv::DECOMP_SVD);
    w.convertTo(dst, ((x.depth() == CV_64F || y.depth() == CV_64F) ? CV_64F : CV_32F));
}

double LaneDetectionModule::getDriveHeading(const Lane& lane1, const Lane& lane2, std::string& direction) {
    double modifiedSlope = 0.0;

    if (lane1.getStatus() && lane2.getStatus()) {
        // Get lane 1
        std::vector<float> laneOneCoeff = lane1.getPolyCoeff();

        // Variables to take the slope of lane 1
        cv::Point2d top, bottom;
        top.y = 20;
        bottom.y = 700;

        // Get top x co-ordinate for y = 20
        top.x = pow(top.y, 2) * laneOneCoeff[2] + pow(top.y, 1) * laneOneCoeff[1] + laneOneCoeff[0];

        // Get bottom x co-ordinate for y = 700;
        bottom.x = pow(bottom.y, 2) * laneOneCoeff[2] + pow(bottom.y, 1) * laneOneCoeff[1] + laneOneCoeff[0];

        // Calculate slope
        double slope = atan((top.y - bottom.y) / (top.x - bottom.x)) * 180 / 3.14159265;

        // We get negative slope to the right and positive slope to the left
        // Thats because of opencv coordinate system
        if (slope > -85 && slope < 0) {
            // Negative case ie turn right. Slope changed to positive for right
            modifiedSlope = 90 + slope;
            direction = "Turn right";
        } else if (slope < 85 && slope > 0) {
            // Positive case ie turn left. Slope changed to negative for left
            modifiedSlope = slope - 90;
            direction = "Turn left";
        } else {
            modifiedSlope = modifiedSlope / 0.5;
            direction = "Head straight";
        }
    }

    return modifiedSlope;
}

LaneDetectionStatus LaneDetectionModule::displayOutput(const cv::Mat& src, cv::Mat& src2, cv::Mat& dst, Lane& lane1, Lane& lane2, cv::Mat inv) {
    std::vector<int> yaxis = { 15, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 715 };

    cv::Mat dispOutput;
    src.copyTo(dispOutput);
    int w = src.cols;
    int h = src.rows;

    // **** Lane 1 **** //
    if (lane1.getStatus()) {
        // Find the points for drawing polynomial
        std::vector<float> laneOneCoeff = lane1.getPolyCoeff();

        // Put in the first point of the lane i.e. starting coordinate
        std::vector<cv::Point> laneOnePoints = { };

        // Iterate through all the points
        for (size_t i = 0; i < yaxis.size(); i++) {
            // Calculate the x values for the y's
            int x = pow(yaxis[i], 2) * laneOneCoeff[2] + pow(yaxis[i], 1) * laneOneCoeff[1] + laneOneCoeff[0];
            laneOnePoints.push_back(cv::Point(x, yaxis[i]));
            cv::circle(dispOutput, cv::Point(x, yaxis[i]), 10, cv::Scalar(125, 0, 125), -1);
            cv::arrowedLine(dispOutput, cv::Point(x + 400, yaxis[i] + 30), cv::Point(x + 400, yaxis[i]), cv::Scalar(255, 255, 0), 6, 8, 0, 0.8);
        }

        // PLot the curve
        const cv::Point *pts1 = (const cv::Point*) cv::Mat(laneOnePoints).data;
        int npts1 = cv::Mat(laneOnePoints).rows;
        cv::polylines(dispOutput, &pts1, &npts1, 1, false, cv::Scalar(153, 255, 153), 10);
    }

    // **** Lane 2 **** //
    if (lane2.getStatus()) {
        // Find the points for drawing polynomial
        std::vector<float> laneTwoCoeff = lane2.getPolyCoeff();
        // Put in the first point of the lane i.e. starting coordinate
        std::vector<cv::Point> laneTwoPoints = { };

        // Iterate through all the points
        for (size_t i = 0; i < yaxis.size(); i++) {
            // Calculate the x values for the y's
            int x = pow(yaxis[i], 2) * laneTwoCoeff[2] + pow(yaxis[i], 1) * laneTwoCoeff[1] + laneTwoCoeff[0];
            laneTwoPoints.push_back(cv::Point(x, yaxis[i]));
            cv::circle(dispOutput, cv::Point(x, yaxis[i]), 10, cv::Scalar(125, 0, 125), -1);
        }

        // PLot the curve
        const cv::Point *pts2 = (const cv::Point*) cv::Mat(laneTwoPoints).data;
        int npts2 = cv::Mat(laneTwoPoints).rows;
        cv::polylines(dispOutput, &pts2, &npts2, 1, false, cv::Scalar(153, 255, 153), 10);
    }

    cv::Mat unwarpedOutput, unwarpedColor;
    cv::warpPerspective(dispOutput, unwarpedOutput, inv, cv::Size(w, h));
    unwarpedOutput.copyTo(unwarpedColor);

    // Combine both the output images (Opencv has a method?)
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if ((unwarpedColor.at<cv::Vec3b>(j, i)[0] == 0)
                && (unwarpedColor.at<cv::Vec3b>(j, i)[1] == 0)
                && (unwarpedColor.at<cv::Vec3b>(j, i)[2] == 0)) {
                unwarpedColor.at<cv::Vec3b>(j, i)[0] = src2.at<cv::Vec3b>(j, i)[0];
                unwarpedColor.at<cv::Vec3b>(j, i)[1] = src2.at<cv::Vec3b>(j, i)[1];
                unwarpedColor.at<cv::Vec3b>(j, i)[2] = src2.at<cv::Vec3b>(j, i)[2];
            }
        }
    }

    // Get drive heading
    std::string direction;
    double heading = getDriveHeading(lane1, lane2, direction);

    // Setting precision to two decimals
    heading = static_cast<int>(100 * heading) / 100.0;
    std::string headStr = std::to_string(heading);
    for (std::string::size_type s = headStr.length() - 1; s > 0; --s) {
        if (headStr[s] == '0')
            headStr.erase(s, 1);
        else
            break;
    }

    std::string result = "Drive angle: " + headStr + " degrees.";
    cv::putText(unwarpedColor, result, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(unwarpedColor, direction, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

    unwarpedColor.copyTo(dst);

    return {
        .frame = dst,
        .direction = direction,
        .steer_angle = std::stod(headStr)
    };
}

LaneDetectionStatus LaneDetectionModule::detect_lane(cv::Mat frame) {
    if (frame.empty()) {
        return {};
    }

    Lane leftLane(2, "red", 10), rightLane(2, "green", 10);

    cv::Mat whiteThreshold, yellowThreshold, combinedThreshold,
        distColor, gaussianBlurImage, ROIImage, warpedImage, undistortedImage,
        laneColor, finalOutput;

    try {
        // Step 1: Undistort the input image given camera params
        undistortImage(frame, undistortedImage);

        // Step 2: Get white lane
        thresholdImageW(undistortedImage, whiteThreshold);

        // Step 3: Get Yellow lane
        thresholdImageY(undistortedImage, yellowThreshold);

        // Step 4: Combine both the masks
        bitwise_or(whiteThreshold, yellowThreshold, combinedThreshold);
        combinedThreshold = ~combinedThreshold;

        // Combine mask with grayscale image. Do we need this?
        cv::Mat grayscaleImage, grayscaleMask;
        cv::cvtColor(undistortedImage, grayscaleImage, cv::COLOR_BGR2GRAY);
        grayscaleImage = ~grayscaleImage;

        bitwise_and(grayscaleImage, combinedThreshold, grayscaleMask);

        // Step 5: Apply Gaussian filter to smooth thresholded image
        cv::GaussianBlur(combinedThreshold, gaussianBlurImage, cv::Size(5, 5), 0, 0);

        // Step 6: Extract the region of interest
        extractROI(gaussianBlurImage, ROIImage);

        // Step 7: Transform perspective
        cv::Mat transformMatrix, invtransformMatrix;
        transformPerspective(ROIImage, warpedImage, transformMatrix, invtransformMatrix);

        // Step 8: Get the lane parameters
        // curveFlag = 1: Straight Line
        // curveFlag = 2: 2nd order polynomial fit
        extractLanes(warpedImage, laneColor, leftLane, rightLane, 2);

        // Adding inverse transform to show in final video
        cv::Mat unwarpedOutput;
        cv::warpPerspective(laneColor, unwarpedOutput, invtransformMatrix, cv::Size(1280, 720));

        // Step 9: Display the output
        return displayOutput(laneColor, frame, finalOutput, leftLane, rightLane, invtransformMatrix);
    } catch (...) {
        std::cerr << "can't detect the lanes" << std::endl;
    }

    return {};
}

cv::Scalar LaneDetectionModule::getYellowMax() {
    return yellowMax_;
}

cv::Scalar LaneDetectionModule::getYellowMin() {
    return yellowMin_;
}

void LaneDetectionModule::setYellowMax(cv::Scalar value) {
    yellowMax_ = value;
}

void LaneDetectionModule::setYellowMin(cv::Scalar value) {
    yellowMin_ = value;
}

void LaneDetectionModule::setGrayScaleMax(int value) {
    grayscaleMax_ = value;
}

void LaneDetectionModule::setGrayScaleMin(int value) {
    grayscaleMin_ = value;
}

int LaneDetectionModule::getGrayScaleMin() {
    return grayscaleMin_;
}

int LaneDetectionModule::getGrayScaleMax() {
    return grayscaleMax_;
}
