#ifndef DRAWER_H
#define DRAWER_H

#include <string>
#include "opencv2/opencv.hpp"

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"
#include "QuadDetector.h"
#include "Marker.h"

using std::string;


class Drawer
{
	void colorAPixel(cv::Mat& img, int x, int y, cv::Scalar color, int dotWidth);

public:
	// draws edge segments
	void drawEdgeMap(const string& path, cv::Mat image, EdgeMap* edgeMap);

	// draws line segments
	void drawLines(const string& path, cv::Mat image, EDLines* edLines);

	// draws corners (intersections of line segments)
	void drawCorners(const string& path, cv::Mat image, const vector<vector<Corner>> &cornerGroups);

	// draws quads
	void drawQuads(const string& path, cv::Mat image, const vector<Quad> &quads);

	// draws markers
	void drawMarkers(const string& path, cv::Mat image, const vector<Marker> &markers);

	// draws refined markers and their ellipses
	void drawEllipses(const string& path, cv::Mat image, const vector<Marker> &markers);
};

#endif