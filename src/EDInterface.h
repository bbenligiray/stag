#ifndef EDINTERFACE_H
#define EDINTERFACE_H

#include "opencv2/opencv.hpp"

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"

namespace stag {

class EDInterface {
  EdgeMap* edgeMap = NULL;
  EDLines* edLines = NULL;

public:
  // runs EDPF and EDLines, keeps the results in memory
  void runEDPFandEDLines(const cv::Mat& image);

  EdgeMap* getEdgeMap();

  EDLines* getEDLines();

  // ensures that when going from the start to end of a line segment, right-hand side is darker
  void correctLineDirection(const cv::Mat& image, LineSegment& ls);

  // calculates the intersection of two line segments
  cv::Point2d intersectionOfLineSegments(const LineSegment& line1, const LineSegment& line2);
};

} // namespace stag

#endif