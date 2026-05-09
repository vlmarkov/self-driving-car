#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

std::vector<double> transform_image_to_1d_histogram(const cv::Mat& image);
cv::Mat convert_to_gray_scale(const cv::Mat& image);
cv::Mat apply_gaussian_blur(const cv::Mat& image);
cv::Mat extract_roi(const cv::Mat& image);
cv::Mat transform_perspective(const cv::Mat& image);
cv::Mat untransform_perspective(const cv::Mat& image);
cv::Mat canny_edge(const cv::Mat& image);
cv::Mat white_threshold(const cv::Mat& image);
