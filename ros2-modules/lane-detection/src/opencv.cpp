#include <lane-detection/opencv.h>

std::vector<double> transform_image_to_1d_histogram(const cv::Mat& image) {
    std::vector<double> histogram;
    // Reduce the input image to a single row (1d histogram)
    cv::reduce(image, histogram, 0, cv::REDUCE_SUM);

    return histogram;
}

cv::Mat convert_to_gray_scale(const cv::Mat& image) {
    cv::Mat gray_scale_image;
    cv::cvtColor(image, gray_scale_image, cv::COLOR_BGR2GRAY);

    return gray_scale_image;
}

cv::Mat apply_gaussian_blur(const cv::Mat& image) {
    cv::Mat gaussian_blur_image; 
    cv::GaussianBlur(image, gaussian_blur_image, cv::Size(5, 5), 0, 0);

    return gaussian_blur_image;
}

cv::Mat extract_roi(const cv::Mat& image) {
    cv::Mat roi_image;

    const auto image_width = image.cols;
    const auto image_height = image.rows;
    const auto bottom_right_corner = cv::Point2f(image_width, image_height);
    const auto bottom_left_corner = cv::Point2f(0, image_height);

    const auto roi_horizontal_line = image_height / 2; // TODO: ???
    const auto roi_vertical_line = image_width * 0.1; // TODO: ???

    const auto roi_upper_left_corner = cv::Point2f(roi_vertical_line, roi_horizontal_line);
    const auto roi_upper_right_corner = cv::Point2f(image_width - roi_vertical_line, roi_horizontal_line);

    // mask matrix initialized as zero Matrix of Height and Width of frame matrix.
    cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8U);

    // Make corners points for the mask.
    // Calibrateted for black and white sample
    cv::Point pts[4] = {
        bottom_right_corner, bottom_left_corner,
        roi_upper_left_corner, roi_upper_right_corner 
    };

    // Create a polygon
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));

    // And the source thresholded image and mask
    cv::bitwise_and(image, mask, roi_image);

    return roi_image;
}

cv::Mat transform_perspective(const cv::Mat& image) {
    cv::Mat transform_image;

    const auto image_width = image.cols;
    const auto image_height = image.rows;
    const auto bottom_right_corner = cv::Point2f(image_width, image_height);
    const auto bottom_left_corner = cv::Point2f(0, image_height);
    const auto upper_left_corner = cv::Point2f(0, 0);
    const auto upper_right_corner = cv::Point2f(image_width, 0);

    const auto roi_horizontal_line = image_height / 2; // TODO: ???
    const auto roi_vertical_line = image_width * 0.1; // TODO: ???

    const auto roi_upper_left_corner = cv::Point2f(roi_vertical_line, roi_horizontal_line);
    const auto roi_upper_right_corner = cv::Point2f(image_width - roi_vertical_line, roi_horizontal_line);

    // Make corners for the transform
    // br, bl, tl, tr -> order
    const cv::Point2f start[4] = {
        bottom_right_corner,
        bottom_left_corner,
        roi_upper_left_corner,
        roi_upper_right_corner,
    };

    // Must be the same as original
    const cv::Point2f end[4] = {
        bottom_right_corner,
        bottom_left_corner,
        upper_left_corner,
        upper_right_corner
    };

    auto tm = cv::getPerspectiveTransform(start, end);
    cv::warpPerspective(image, transform_image, tm, cv::Size(image_width, image_height));

    return transform_image;
}

cv::Mat untransform_perspective(const cv::Mat& image) {
    cv::Mat untransform_image;

    const auto image_width = image.cols;
    const auto image_height = image.rows;
    const auto bottom_right_corner = cv::Point2f(image_width, image_height);
    const auto bottom_left_corner = cv::Point2f(0, image_height);
    const auto upper_left_corner = cv::Point2f(0, 0);
    const auto upper_right_corner = cv::Point2f(image_width, 0);

    const auto roi_horizontal_line = image_height / 2; // TODO: ???
    const auto roi_vertical_line = image_width * 0.1; // TODO: ???

    const auto roi_upper_left_corner = cv::Point2f(roi_vertical_line, roi_horizontal_line);
    const auto roi_upper_right_corner = cv::Point2f(image_width - roi_vertical_line, roi_horizontal_line);
    // Make corners for the transform
    // br, bl, tl, tr -> order
    const cv::Point2f start[4] = {
        bottom_right_corner,
        bottom_left_corner,
        roi_upper_left_corner,
        roi_upper_right_corner,
    };

    // Must be the same as original
    const cv::Point2f end[4] = {
        bottom_right_corner,
        bottom_left_corner,
        upper_left_corner,
        upper_right_corner
    };

    auto inv_tm = cv::getPerspectiveTransform(end, start);

    cv::warpPerspective(image, untransform_image, inv_tm, cv::Size(image_width, image_height));

    return untransform_image;
}

cv::Mat canny_edge(const cv::Mat& image) {
    cv::Mat edges;
    cv::Canny(image, edges, 10, 100);

    return edges;
}

cv::Mat white_threshold(const cv::Mat& image) {
    const int grayscale_min{195}; // A white lane min threshold
    const int grayscale_max{255}; // A white lane max threshold

    cv::Mat white_threshold;
    cv::Mat combined_threshold;
    cv::threshold(image, white_threshold, grayscale_min, grayscale_max, cv::THRESH_BINARY);

    cv::bitwise_or(white_threshold, white_threshold, combined_threshold);
    combined_threshold = ~combined_threshold;

    return combined_threshold;
}
