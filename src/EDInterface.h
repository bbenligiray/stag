#ifndef EDINTERFACE_H
#define EDINTERFACE_H

#include <opencv2/opencv.hpp>

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"

namespace stag {

class EDInterface {
  edpf::EdgeMap* edgeMap = NULL;
  edpf::EDLines* edLines = NULL;

public:
  // runs EDPF and EDLines, keeps the results in memory
  void runEDPFandEDLines(const cv::Mat& image);

  edpf::EdgeMap* getEdgeMap();

  edpf::EDLines* getEDLines();

  // ensures that when going from the start to end of a line segment, right-hand side is darker
  void correctLineDirection(const cv::Mat& image, edpf::LineSegment& ls);

  // calculates the intersection of two line segments
  cv::Point2d intersectionOfLineSegments(const edpf::LineSegment& line1, const edpf::LineSegment& line2);
};

} // namespace stag

#endif
