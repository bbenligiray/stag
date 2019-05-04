#ifndef STAG_H
#define STAG_H

#include "EDInterface.h"
#include "QuadDetector.h"
#include "Drawer.h"
#include "Marker.h"
#include "Decoder.h"
#include "PoseRefiner.h"

class Stag
{
	// if keepLogs is true, keep the intermediate results of the detection algorithm in the memory, to be dumped when asked (default: false)
	bool keepLogs = false;
	int errorCorrection;
	EDInterface edInterface;
	QuadDetector quadDetector;
	Drawer drawer;
	Decoder decoder;
	PoseRefiner poseRefiner;

	vector<cv::Mat> codeLocs;
	vector<cv::Mat> blackLocs;
	vector<cv::Mat> whiteLocs;

	cv::Mat image;
	vector<Marker> markers;
	vector<Quad> falseCandidates;

	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	// thresholds and converts to binary code
	Codeword readCode(const Quad &q);
	void fillCodeLocations();
	cv::Mat createMatFromPolarCoords(double radius, double radians, double circleRadius);
public:
	Stag(int libraryHD = 15, int errorCorrection = 7, bool inKeepLogs = false);
	void detectMarkers(cv::Mat inImage);
	void logResults(string path = "");
};


#endif