#include "Stag.h"

#include "opencv2/opencv.hpp"

int main() {
  cv::Mat image = cv::imread("1.png", cv::IMREAD_GRAYSCALE);

  Stag stag(15, 7, true);

  stag.detectMarkers(image);
  stag.logResults("");
  return 0;
}