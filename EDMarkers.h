#ifndef EDMARKERS_H
#define EDMARKERS_H

#include <vector>
#include <string>
#include "opencv2/opencv.hpp"

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"
#include "CommonClasses.h"
#include "Decoder.h"
#include "Marker.h"
#include "Ellipse.h"
#include "ED/Timer.h"
#include "CurvatureCornerDetection.h"

using std::vector;
using std::string;
using cv::Mat;

class EDMarkers
{
	Mat image;
	int width, height;
	bool initialized = false;

	// if this flag is set, create a paper trail of the detection process in the memory (def: false)
	bool keepPaperTrail = false;



	// points to be sampled for brightness
	static vector<Mat> codeLocs;
	static vector<Mat> blackLocs;
	static vector<Mat> whiteLocs;

	// object for holding the codewords and decoding the read bit sequence
	static Decoder decoder;

	// 0 - entire time
	vector<double> timings;



	// ---these are always saved---
	EDLines* edLines;
	EdgeMap* edgeMap;
	vector<vector<Corner>> cornerGroups;
	vector<vector<int>> curvCornerInds;
	vector<Marker> markers;
	int noOfCandidates = 0;
	// ---these are always saved---



	// ---these are only saved if keepPaperTrail == true---
	vector<Marker> roughMarkers;
	vector<Marker> distMarkers;
	vector<Marker> invalidMarkers;
	vector<Point2d> arcPixels;
	// ---these are only saved if keepPaperTrail == true---

	

	// ---parameters---

	// thresManhDist is used for following purposes. may need to increase it for higher resolutions (def: 10)
	// to check if edge segments form loops
	// if detected corners are the same with estimated corners
	// if detected corners are on edge segments
	int thresManhDist = 10;

	// uses only loop edge segments. to be used when no occlusion is expected, e.g. calibration (def: false)
	bool flagOnlyUseEdgeSegmentLoops = false;

	// this forms quads using any 4 combination of lines. only to be used for experimental purposes (def: false)
	bool flagFullyOcclusionResistant = false;

	// only detects quads with darker interior (def: true)
	bool flagCorrectLineDirections = true;

	// checks if detected corners are on edge segments to eliminate corners detected on curves (def: true)
	bool flagCheckCornerDistToEdges = true;

	// checks if quads are distorted (def: true)
	bool flagCheckProjectiveDistortion = true;

	// projective distortion threshold (def: 1.5)
	double thresProjDist = 1.5;

	// detects ellipse and refines pose (def: true)
	bool flagRefinePose = true;

	// number of bits to be corrected. maximum value is errCorr * 2 + 1 = HD (def: 7)
	int errCorr = 7;

	// ---parameters---



	// initializes static code locations to be sampled
	void fillCodeLocations();

	// decodes a candidate, and returns true if it's a match
	bool decodeCandidate(Marker& cand);

	// refines homography using an ellipse detection. returns false if ellipse is not located and refinement is not done
	bool refinePoseEstimation(Marker& cand);

	// changes the line segment directions such that right hand side is darker when going from start to end
	void correctLineDirection(LineSegment& ls);

	// normal (non-occlusion resistant) detection method only checks consecutive line segments on the edge segments
	// this (occlusion resistant) detection method checks all two combinations of lines to detect corners
	// any two corneres that face each other form a candidate
	void occlusionResistantDetection();

	void cleanUp();



	// use this when the pixel is guaranteed to be in the image boundaries
	unsigned char readPixelUnsafe(const Point2i& p);
	// use this when at least one of the pixels to be read is outside image boundaries
	unsigned char readPixelSafe(const Point2i& p);
	// use this when the read brightness value is critical, e.g. when decoding
	unsigned char readPixelSafeBilinear(const Point2d& p);


	// features from the same edge segments are drawn with the same color
	static vector<cv::Scalar> colors;

	void colorAPixel(Mat& img, int x, int y, cv::Scalar color, int dotWidth);
	// draws edge segments
	void drawEdgeMap(const string& path, const string& name);
	// draws line segments
	void drawLines(const string& path, const string& name);
	// draws corners (intersections of line segments)
	void drawCorners(const string& path, const string& name);
	// draws marker detections
	void drawMarkers(const string& path, const string& name);
	// draws markers with invalid codes
	void drawInvalidMarkers(const string& path, const string& name);
	// draws markers with excessive projective distortion
	void drawDistMarkers(const string& path, const string& name);

	// draws unrefined marker detections
	void drawRoughMarkers(const string& path, const string& name);
	// draws all arcs, curvature corners and shows the chosen arcs with bright green
	void drawArcs(const string& path, const string& name);
	// draws ellipses
	void drawEllipses(const string& path, const string& name);

	// outputs results at a text file
	void writeResults(const string& path, const string& name);

public:
	EDMarkers(int libraryHD, int errorCorrection);

	// keeps intermediate steps in memory if debug == true. call dumpLog() to get output.
	void detectMarkers(Mat image, bool debug);

	// deletes ED related stuff
	~EDMarkers();

	// call this function before using the marker system
	// if you skip to do so, this function is called with HD = 15 errCorr = 7
	// this function can also be used to switch between marker libraries
	void initialize(int libraryHD, int errorCorrection);

	// outputs everyting at path, with file names starting with name (to batch process frames)
	// (TODO: what are logged exactly?)
	void dumpLog(const string& path, const string& name);

	vector<Marker> getMarkers();

	vector<Marker> getRoughMarkers();

	vector<double> getTimings();

	EDLines* getLines();

	EdgeMap* getEdgeMap();

	void setRefinementFlag(bool refine);
};

#endif