#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

namespace stag {

void detectMarkers( const cv::Mat& image,
                    int libraryHD,
                    std::vector<std::vector<cv::Point2f>>& output_corners,
                    std::vector<int>& output_ids,
                    int error_correction = 7 );

void drawDetectedMarkers( cv::Mat& image,
                          const std::vector<std::vector<cv::Point2f>>& corners,
                          const std::vector<int>& ids,
                          const cv::Scalar& border_color = cv::Scalar(50, 255, 50) );

} // namespace stag
