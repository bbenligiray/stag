#ifndef STAG_H
#define STAG_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "Decoder.h"
#include "Drawer.h"
#include "EDInterface.h"
#include "Marker.h"
#include "PoseRefiner.h"
#include "QuadDetector.h"

namespace stag {

class Stag {
  // if keepLogs is true, keep the intermediate results of the detection algorithm in the memory, to be dumped when asked (default: false)
  bool keepLogs = false;
  int errorCorrection;
  EDInterface edInterface;
  QuadDetector quadDetector;
  Drawer drawer;
  Decoder decoder;
  PoseRefiner poseRefiner;

  std::vector<cv::Mat> codeLocs;
  std::vector<cv::Mat> blackLocs;
  std::vector<cv::Mat> whiteLocs;

  cv::Mat image;
  std::vector<Marker> markers;
  std::vector<Quad> falseCandidates;

  // take readings from 48 code locations, 12 black border locations, and 12 white border locations
  // thresholds and converts to binary code
  Codeword readCode(const Quad& q);
  void fillCodeLocations();
  cv::Mat createMatFromPolarCoords(double radius, double radians, double circleRadius);
public:
  Stag(int libraryHD = 15, int errorCorrection = 7, bool inKeepLogs = false);
  void detectMarkers(cv::Mat inImage);
  void logResults(std::string path = "");

  const std::vector<Marker>& getMarkers();
};

} // namespace stag


#endif