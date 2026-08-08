#include <lane-detection/lane_detection_module.h>
#include <lane-detection/opencv.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace {

std::vector<cv::Point> extract_one_lane(
    const cv::Mat& image,
    cv::Mat& dst,
    const int window_count,
    const int window_height,
    const int window_width,
    const int image_width,
    const int image_height,
    const int padding,
    const int bottom_height,
    const int bottom_width,
    size_t max_index
) {
    std::vector<cv::Point> lane_points;

    auto current_height = bottom_height;
    for (int i = 0; i < window_count; i++) {
        // Get the top left and bottom right point to make a rectangle
        int tlX = static_cast<int>(max_index) - window_width / 2;
        int tlY = current_height - window_height;
        int brX = static_cast<int>(max_index) + window_width / 2;
        int brY = current_height;

        // Check boundary
        tlX = (tlX < 0) ? padding : tlX;
        tlY = (tlY < 0) ? padding : tlY;

        brX = (brX > image_width) ? bottom_width : brX;
        brY = (brY > image_height) ? bottom_height : brY;

        // Create a temporary vector to store the x's for next box
        std::vector<cv::Point> points;
        const int points_density = 1000;
        int cnt = 0;

        for (auto x = tlX; x <= brX; x++) {
            for (auto y = tlY; y <= brY; y++) {  
                if (static_cast<int>(image.at<uchar>(y, x)) == 255) {
                    cnt++;
                    if (cnt % points_density) {
                        cnt = 0;
                        points.push_back(cv::Point{x, y});
                    }
                }
            }
        }

        cv::Point slide_window_point{-1, -1};

        if (!points.empty()) {
            auto bottom = std::max_element(points.begin(), points.end(), [](const auto& a, const auto& b) {
                return a.y < b.y;
            });
            auto top = std::max_element(points.begin(), points.end(), [](const auto& a, const auto& b) {
                return a.y > b.y;
            });

            cv::rectangle(dst, {bottom->x, bottom->y}, {top->x, top->y}, cv::Scalar(0, 255, 204), 1);

            cv::rectangle(
                dst,
                {(bottom->x + top->x)/2, (bottom->y + top->y)/2},
                {(bottom->x + top->x)/2, (bottom->y + top->y)/2},
                cv::Scalar(0, 255, 204),
                10
            );

            slide_window_point.x = (bottom->x + top->x)/2;
            slide_window_point.y = (bottom->y + top->y)/2;
            max_index = static_cast<size_t>(top->x);
        }

        lane_points.push_back(slide_window_point);
        current_height -= window_height;
    }

    return lane_points;
}

void visualize_trajectory_lane_points(cv::Mat& image, const std::vector<cv::Point>& trajectory) {
    for (const auto& p : trajectory) {
        cv::rectangle(image, {p.x, p.y}, {p.x, p.y}, cv::Scalar(0, 255, 204), 20);
    }
}

/******************************************************************************/
/*   .     .           .       .      .        .                              */
/*   .     .          .       .        .        .                             */
/*   .     .         .       .          .        .                            */
/*   .     .        .       .            .        .                           */
/*   .     .       .       .              .        .                          */
/*   .     .      .       .                .        .                         */
/*   .     .      .       .                 .        .                        */
/******************************************************************************/

/******************************************************************************/
/*         .           .       .      .        .              .               */
/*         .                  .        .        .             .               */
/*         .                 .          .        .            .               */
/*         .                .            .                    .               */
/*         .               .              .                   .               */
/*   .     .      .       .                .                  .               */
/*   .     .      .       .                 .                 .               */
/******************************************************************************/

bool is_point_invalid(const cv::Point& p) {
    return p.x == -1 && p.y == -1;
}

bool is_point_valid(const cv::Point& p) {
    return p.x != -1 && p.y != -1;
}

cv::Point get_or(
    const cv::Point& target_point,
    const cv::Point& backup_point,
    const int default_x_offset
) {
    if (is_point_valid(target_point)) {
        return target_point;
    }

    cv::Point result;

    result.x = backup_point.x + (2  * default_x_offset);
    result.y = backup_point.y;

    return result;
}

std::vector<cv::Point> extract_trajectory_lane_points(
    const std::vector<cv::Point>& left,
    const std::vector<cv::Point>& right
) {
    if (left.size() != right.size()) {
        return {};
    }

    std::vector<cv::Point> trajectory_points;

    auto last_middle_x_offset = 100; // TODO: !

    for (size_t i = 0; i < left.size(); ++i) {
        if (is_point_invalid(left[i]) && is_point_invalid(right[i])) {
            continue;
        }

        const auto& left_point = get_or(left[i], right[i], -last_middle_x_offset);
        const auto& right_point = get_or(right[i], left[i], last_middle_x_offset);

        const auto middle_x = (right_point.x + left_point.x) / 2;
        const auto middle_y = (right_point.y + left_point.y) / 2;

        last_middle_x_offset = std::abs(right_point.x - middle_x);

        trajectory_points.push_back(cv::Point(middle_x, middle_y));
    }

    return trajectory_points;
}

std::optional<int> get_drive_angle(const std::vector<cv::Point>& points) {
    // https://www.kaggle.com/code/sujaykapadnis/lane-detection-for-self-driving-cars
    // https://github.com/RobuRishabh/Lane_detector_Using_OpenCV_for_autonomous_vehicle
    // https://github.com/tatsuyah/Lane-Lines-Detection-Python-OpenCV
    // https://github.com/hayoung-kim/Perception-for-Self-driving
    if (points.empty()) {
        return {};
    }
    if (points.size() == 1) {
        return {};
    }

    const auto& bottom = points.front();
    const auto& top = points.back();
    const float slope = (top.x - bottom.x) != 0 ? std::atan((top.y - bottom.y) / (top.x - bottom.x)) * (180 / 3.14159265) : 0.0;

    return slope < 0 ? -(90 + slope) : (90 - slope);
}

std::pair<std::vector<cv::Point>, cv::Mat> extract_lanes(const cv::Mat& image, const cv::Mat& color_image) {
    cv::Mat dst = color_image.clone();

    const auto max_threshold = 1000.0;

    const auto image_width = image.cols;
    const auto image_height = image.rows;

    // Height to start the sliding windows
    const int padding = 40;
    const int bottom_height = image_height - padding;
    const int bottom_width = image_width - padding;

    // Sliding Window approach
    const int window_count = 10;
    const int window_height = (image_height - padding) / window_count;
    const int window_width = window_height * 2;

    // Create a mask for bottom 1/4 of image to get the histograms
    const cv::Rect roi(
        0,                                   // start x
        (image_height - (image_height / 4)), // start y
        image_width ,                        // width
        image_height / 4                     // height
    );
    const cv::Mat cropped_im = image(roi);
    // Reduce the input matrix to a single row
    const auto histogram = transform_image_to_1d_histogram(cropped_im);
    if (histogram.empty() || histogram.size() < 2) {
        return {};
    }

    // Split the two vectors for left and right lane
    const std::size_t half_size = histogram.size() / 2;
    const auto border_offset = 1;
    const std::vector<double> left_hist(histogram.begin() + border_offset, histogram.begin() + half_size);
    const std::vector<double> right_hist(histogram.begin() + half_size, histogram.end() - 1 - border_offset);

    // Get the max element in left half
    const size_t max_left_index = std::distance(left_hist.begin(), std::max_element(left_hist.begin(), left_hist.end()));
    // Get the max element in right half
    const size_t max_right_index = std::distance(right_hist.begin(), std::max_element(right_hist.begin(), right_hist.end())) + half_size;

    std::vector<cv::Point> left_lane_points;
    std::vector<cv::Point> right_lane_points;

    for (int i = 0; i < window_count; i++) {
        left_lane_points.push_back(cv::Point{-1, -1});
        right_lane_points.push_back(cv::Point{-1, -1});
    }

    if (histogram.at(max_left_index) > max_threshold) {
        left_lane_points = extract_one_lane(
            image, dst, window_count, window_height, window_width,
            image_width, image_height, padding, bottom_height, bottom_width,
            max_left_index
        );
    }

    if (histogram.at(max_right_index) > max_threshold) {
        right_lane_points = extract_one_lane(
            image, dst, window_count, window_height, window_width,
            image_width, image_height, padding, bottom_height, bottom_width,
            max_right_index
        );
    }

    auto result = extract_trajectory_lane_points(left_lane_points, right_lane_points);

    visualize_trajectory_lane_points(dst, result);

    return {result, dst};
}

} // namespace

LaneDetectionModule::LaneDetectionModule(LaneDetectionCfg) {
    // Nothing so far
}

std::optional<LaneDetectionStatus> LaneDetectionModule::detect_lane(const cv::Mat& frame) {
    if (frame.empty()) {
        return {};
    }

    const auto gray_scale_frame = convert_to_gray_scale(frame);
    const auto gaussian_blur_frame = apply_gaussian_blur(gray_scale_frame);
    const auto roi_frame = extract_roi(gray_scale_frame);
    const auto transform_perspective_frame = transform_perspective(roi_frame);
    const auto white_threshold_frame = canny_edge(transform_perspective_frame);

    const auto [extracted_lanes, image] = extract_lanes(white_threshold_frame, transform_perspective_frame);
    const auto angle = get_drive_angle(extracted_lanes);
    if (!angle.has_value()) {
        return {};
    }

    std::string direction = "Head straight";
    const int forward_deviation_angle = 3;
    if ((angle.value() < 0) && (std::abs(angle.value()) >= forward_deviation_angle)) {
        direction = "Turn right";
    } else if ((angle.value() > 0) && (std::abs(angle.value()) >= forward_deviation_angle)) {
        direction = "Turn left";
    }

    return LaneDetectionStatus{
        .frame = image,
        .direction = direction,
        .steer_angle= std::abs(angle.value()) >= forward_deviation_angle ? angle.value() : 0
    };
}
