#ifndef POSEREFINER_H
#define POSEREFINER_H

#include "opencv2/opencv.hpp"

#include "EDInterface.h"
#include "Marker.h"

namespace stag {

class PoseRefiner
{
	bool checkIfPointInQuad(const Marker& marker, const cv::Point2d& p);
	cv::Point2d projectPoint(cv::Point2d p, cv::Mat H);
public:
	void refineMarkerPose(EDInterface* edInterface, Marker& marker);
};

} // namespace stag

#endif