#include "stag/StagMarkers.h"

#include "Stag.h"

namespace stag {

void detectMarkers( const cv::Mat& image,
                    int libraryHD,
                    std::vector<std::vector<cv::Point2f>>& output_corners,
                    std::vector<int>& output_ids,
                    int error_correction ) {

  output_corners.clear();
  output_ids.clear();

  stag::Stag stag_detector(libraryHD, error_correction, false);
  stag_detector.detectMarkers(image);
  const auto& markers = stag_detector.getMarkers();

  for (const auto& marker : markers) {
    std::vector<cv::Point2f> marker_corners;
    std::transform(marker.corners.begin(),
                   marker.corners.end(),
                   std::back_inserter(marker_corners),
                   [](const cv::Point2d& pt_d) { return cv::Point2f(static_cast<float>(pt_d.x), static_cast<float>(pt_d.y)); } );

    output_ids.push_back(marker.id);
    output_corners.emplace_back(std::move(marker_corners));
  }
}

void drawDetectedMarkers( cv::Mat& image,
                          const std::vector<std::vector<cv::Point2f>>& corners,
                          const std::vector<int>& ids,
                          const cv::Scalar& border_color ) {

  uint markers_number = ids.size();

  for (uint i = 0; i < markers_number; i++) {
    const std::vector<cv::Point2f>& marker_corners = corners[i];
    const int& marker_id = ids[i];

    cv::circle(image, cv::Point(static_cast<int>(marker_corners[0].x), static_cast<int>(marker_corners[0].y)), 6, cv::Scalar(255, 255, 255), -1);
    for (int j = 0; j < 4; j++)
      cv::line(image, cv::Point(static_cast<int>(marker_corners[j].x), static_cast<int>(marker_corners[j].y)), cv::Point(static_cast<int>(marker_corners[(j + 1) % 4].x), static_cast<int>(marker_corners[(j + 1) % 4].y)), cv::Scalar(255, 255, 255), 3);

    cv::circle(image, cv::Point(static_cast<int>(marker_corners[0].x), static_cast<int>(marker_corners[0].y)), 5, border_color, -1);
    for (int j = 0; j < 4; j++)
      cv::line(image, cv::Point(static_cast<int>(marker_corners[j].x), static_cast<int>(marker_corners[j].y)), cv::Point(static_cast<int>(marker_corners[(j + 1) % 4].x), static_cast<int>(marker_corners[(j + 1) % 4].y)), border_color, 2);

    cv::Point text_pos(static_cast<int>((marker_corners[0].x + marker_corners[2].x) / 2), static_cast<int>((marker_corners[0].y + marker_corners[2].y) / 2));
    cv::putText(image, std::to_string(marker_id), text_pos, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 5);
    cv::putText(image, std::to_string(marker_id), text_pos, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(50, 50, 255), 2);
  }
}

} // namespace stag
