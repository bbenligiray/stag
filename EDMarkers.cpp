#include <algorithm>
#include <iostream>
#include <fstream>
#include <numeric>
#include "EDMarkers.h"
#include "ColorPalette.h"

using std::min;
using std::max;
using std::abs;

// these have to be declared here again because they are static
vector<Mat> EDMarkers::codeLocs;
vector<Mat> EDMarkers::blackLocs;
vector<Mat> EDMarkers::whiteLocs;
Decoder EDMarkers::decoder;
vector<cv::Scalar> EDMarkers::colors;

// global variables for LM
int LMFunc(int m, int n, double *p, double *deviates, double **derivs, void *privateData);
Mat LMellipse;
vector<Mat> LMcornerPixels;

class Refine :public cv::MinProblemSolver::Function{
public:
	int getDims() const { return 9; }
	double calc(const double* x)const{
		Mat H, HT, Hi;
		H = Mat(3, 3, CV_64FC1);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				H.at<double>(i, j) = x[i + j * 3];
		}


		cv::transpose(H, HT);
		Hi = H.inv();

		// project the detected ellipse using the current homography matrix
		Mat projCircle = HT * LMellipse * H;

		vector<double> projEllipseCoeff(6);
		projEllipseCoeff[0] = projCircle.at<double>(0, 0);
		projEllipseCoeff[1] = -projCircle.at<double>(0, 1) * 2;
		projEllipseCoeff[2] = projCircle.at<double>(1, 1);
		projEllipseCoeff[3] = projCircle.at<double>(0, 2) * 2;
		projEllipseCoeff[4] = -projCircle.at<double>(1, 2) * 2;
		projEllipseCoeff[5] = projCircle.at<double>(2, 2);

		// construct the ellipse using coefficients (to calculate semi major/minor axes)
		customEllipse projEllipse(projEllipseCoeff.data());



		vector<double> errors;

		errors.push_back(abs(projEllipse.GetSemiMajorAxis() - 0.4));
		errors.push_back(abs(projEllipse.GetSemiMinorAxis() - 0.4));
		errors.push_back(abs(projEllipse.GetCenterX() - 0.5));
		errors.push_back(abs(projEllipse.GetCenterY() - 0.5));

		return std::accumulate(errors.begin(), errors.end(), (double)0);
	}
};



void EDMarkers::fillCodeLocations()
{
	// fill coordinates to be sampled
	codeLocs = vector<Mat>(48);

	// code circles are located in a circle with radius outerCircleRadius
	double outerCircleRadius = 0.4;
	double innerCircleRadius = outerCircleRadius * 0.9;

	// each quadrant is rotated by HALF_PI
	// these part is left as is for self-documenting purposes
	for (int i = 0; i < 4; i++)
	{
		codeLocs[0 + i * 12] = createMatFromPolarCoords(0.088363142525988, 0.785398163397448 + i * HALF_PI, innerCircleRadius);

		codeLocs[1 + i * 12] = createMatFromPolarCoords(0.206935928182607, 0.459275804122858 + i * HALF_PI, innerCircleRadius);
		codeLocs[2 + i * 12] = createMatFromPolarCoords(0.206935928182607, HALF_PI - 0.459275804122858 + i * HALF_PI, innerCircleRadius);

		codeLocs[3 + i * 12] = createMatFromPolarCoords(0.313672146827381, 0.200579720495241 + i * HALF_PI, innerCircleRadius);
		codeLocs[4 + i * 12] = createMatFromPolarCoords(0.327493143484516, 0.591687617505840 + i * HALF_PI, innerCircleRadius);
		codeLocs[5 + i * 12] = createMatFromPolarCoords(0.327493143484516, HALF_PI - 0.591687617505840 + i * HALF_PI, innerCircleRadius);
		codeLocs[6 + i * 12] = createMatFromPolarCoords(0.313672146827381, HALF_PI - 0.200579720495241 + i * HALF_PI, innerCircleRadius);

		codeLocs[7 + i * 12] = createMatFromPolarCoords(0.437421957035861, 0.145724938287167 + i * HALF_PI, innerCircleRadius);
		codeLocs[8 + i * 12] = createMatFromPolarCoords(0.437226762361658, 0.433363129825345 + i * HALF_PI, innerCircleRadius);
		codeLocs[9 + i * 12] = createMatFromPolarCoords(0.430628029742607, 0.785398163397448 + i * HALF_PI, innerCircleRadius);
		codeLocs[10 + i * 12] = createMatFromPolarCoords(0.437226762361658, HALF_PI - 0.433363129825345 + i * HALF_PI, innerCircleRadius);
		codeLocs[11 + i * 12] = createMatFromPolarCoords(0.437421957035861, HALF_PI - 0.145724938287167 + i * HALF_PI, innerCircleRadius);
	}

	double borderDist = 0.045;

	blackLocs = vector<Mat>(12);
	whiteLocs = vector<Mat>(12);

	for (int i = 0; i < 12; i++)
		blackLocs[i] = Mat(3, 1, CV_64FC1);
	for (int i = 0; i < 12; i++)
		whiteLocs[i] = Mat(3, 1, CV_64FC1);

	blackLocs[0].at<double>(0) = borderDist;
	blackLocs[0].at<double>(1) = borderDist * 3;
	blackLocs[0].at<double>(2) = 1;

	blackLocs[1].at<double>(0) = borderDist * 2;
	blackLocs[1].at<double>(1) = borderDist * 2;
	blackLocs[1].at<double>(2) = 1;

	blackLocs[2].at<double>(0) = borderDist * 3;
	blackLocs[2].at<double>(1) = borderDist;
	blackLocs[2].at<double>(2) = 1;

	blackLocs[3].at<double>(0) = 1 - 3 * borderDist;
	blackLocs[3].at<double>(1) = borderDist;
	blackLocs[3].at<double>(2) = 1;

	blackLocs[4].at<double>(0) = 1 - 2 * borderDist;
	blackLocs[4].at<double>(1) = borderDist * 2;
	blackLocs[4].at<double>(2) = 1;

	blackLocs[5].at<double>(0) = 1 - borderDist;
	blackLocs[5].at<double>(1) = borderDist * 3;
	blackLocs[5].at<double>(2) = 1;

	blackLocs[6].at<double>(0) = 1 - borderDist;
	blackLocs[6].at<double>(1) = 1 - 3 * borderDist;
	blackLocs[6].at<double>(2) = 1;

	blackLocs[7].at<double>(0) = 1 - 2 * borderDist;
	blackLocs[7].at<double>(1) = 1 - 2 * borderDist;
	blackLocs[7].at<double>(2) = 1;

	blackLocs[8].at<double>(0) = 1 - 3 * borderDist;
	blackLocs[8].at<double>(1) = 1 - borderDist;
	blackLocs[8].at<double>(2) = 1;

	blackLocs[9].at<double>(0) = borderDist * 3;
	blackLocs[9].at<double>(1) = 1 - borderDist;
	blackLocs[9].at<double>(2) = 1;

	blackLocs[10].at<double>(0) = borderDist * 2;
	blackLocs[10].at<double>(1) = 1 - 2 * borderDist;
	blackLocs[10].at<double>(2) = 1;

	blackLocs[11].at<double>(0) = borderDist;
	blackLocs[11].at<double>(1) = 1 - 3 * borderDist;
	blackLocs[11].at<double>(2) = 1;


	whiteLocs[0].at<double>(0) = 0.25;
	whiteLocs[0].at<double>(1) = -borderDist;
	whiteLocs[0].at<double>(2) = 1;

	whiteLocs[1].at<double>(0) = 0.5;
	whiteLocs[1].at<double>(1) = -borderDist;
	whiteLocs[1].at<double>(2) = 1;

	whiteLocs[2].at<double>(0) = 0.75;
	whiteLocs[2].at<double>(1) = -borderDist;
	whiteLocs[2].at<double>(2) = 1;

	whiteLocs[3].at<double>(0) = 1 + borderDist;
	whiteLocs[3].at<double>(1) = 0.25;
	whiteLocs[3].at<double>(2) = 1;

	whiteLocs[4].at<double>(0) = 1 + borderDist;
	whiteLocs[4].at<double>(1) = 0.5;
	whiteLocs[4].at<double>(2) = 1;

	whiteLocs[5].at<double>(0) = 1 + borderDist;
	whiteLocs[5].at<double>(1) = 0.75;
	whiteLocs[5].at<double>(2) = 1;

	whiteLocs[6].at<double>(0) = 0.75;
	whiteLocs[6].at<double>(1) = 1 + borderDist;
	whiteLocs[6].at<double>(2) = 1;

	whiteLocs[7].at<double>(0) = 0.5;
	whiteLocs[7].at<double>(1) = 1 + borderDist;
	whiteLocs[7].at<double>(2) = 1;

	whiteLocs[8].at<double>(0) = 0.25;
	whiteLocs[8].at<double>(1) = 1 + borderDist;
	whiteLocs[8].at<double>(2) = 1;

	whiteLocs[9].at<double>(0) = -borderDist;
	whiteLocs[9].at<double>(1) = 0.75;
	whiteLocs[9].at<double>(2) = 1;

	whiteLocs[10].at<double>(0) = -borderDist;
	whiteLocs[10].at<double>(1) = 0.5;
	whiteLocs[10].at<double>(2) = 1;

	whiteLocs[11].at<double>(0) = -borderDist;
	whiteLocs[11].at<double>(1) = 0.25;
	whiteLocs[11].at<double>(2) = 1;
}

bool EDMarkers::decodeCandidate(Marker& cand)
{
	noOfCandidates++;

	// estimate the pose using the fast pose estimation method
	cand.estimatePose();

	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	vector<unsigned char> readings(72);

	for (int i = 0; i < 48; i++)
		readings[i] = readPixelSafeBilinear(cand.projectPoint(codeLocs[i]));
	for (int i = 0; i < 12; i++)
		readings[i + 48] = readPixelSafeBilinear(cand.projectPoint(blackLocs[i]));
	for (int i = 0; i < 12; i++)
		readings[i + 60] = readPixelSafeBilinear(cand.projectPoint(whiteLocs[i]));

	// threshold the readings using Otsu's method
	cv::threshold(readings, readings, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY_INV);

	// create a codeword using the thresholded readings
	Codeword c;
	for (int i = 0; i < 48; i++)
		c[i] = readings[i] / 255;

	// decode the marker. return false if it's not a match
	// it returns an id and a shift value. this value can be 0-1-2-3 where each corresponds to a 90 degree rotation
	int shift;
	if (!decoder.decode(c, errCorr, cand.id, shift))
		return false;

	// reorder corners according to the shift returned by decode and reestimate pose
	cand.shiftCorners(shift);

	return true;
}

bool EDMarkers::refinePoseEstimation(Marker& cand)
{
	// there is a lot of technical debt here because of the older ED structure, sorry in advance
	static const double sinVals[36] = { 0.000000, 0.173648, 0.342020, 0.500000, 0.642788, 0.766044, 0.866025, 0.939693, 0.984808, 1.000000, 0.984808, 0.939693, 0.866025, 0.766044, 0.642788, 0.500000, 0.342020, 0.173648, 0.000000, -0.173648, -0.342020, -0.500000, -0.642788, -0.766044, -0.866025, -0.939693, -0.984808, -1.000000, -0.984808, -0.939693, -0.866025, -0.766044, -0.642788, -0.500000, -0.342020, -0.173648 };

	customEllipse borderEllipse;
	vector<pix> accPixs;

	Mat Hinv = cand.H.inv();

	vector<Point2d> samplePoints(36);
	for (int i = 0; i < 36; i++)
		samplePoints[i] = Point2d(0.5 + 0.4 * sinVals[(i + 9) % 36], 0.5 + 0.4 * sinVals[i]);

	int minimumArcLength = 20;
	double lowestError = INFINITY;
	double maxError = 36 * 0.05;
	int chosenEdgeSegment = -1;

	for (int i = 0; i < edgeMap->noSegments; i++)
	{
		// skip if the edge segment is too short
		if (edgeMap->segments[i].noPixels < minimumArcLength)
			continue;

		// skip if the edge segment is not a loop
		if (manhDistOfTwoPoints(Point2d(edgeMap->segments[i].pixels[0].c, edgeMap->segments[i].pixels[0].r), Point2d(edgeMap->segments[i].pixels[edgeMap->segments[i].noPixels - 1].c, edgeMap->segments[i].pixels[edgeMap->segments[i].noPixels - 1].r)) > thresManhDist)
			continue;

		// skip if the edge segment is not inside candidate
		bool skipBecauseOutside = false;
		for (int j = 0; j < edgeMap->segments[i].noPixels; j += minimumArcLength)
		{
			if (!cand.checkIfPointInQuad(Point2d(edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r)))
			{
				skipBecauseOutside = true;
				break;
			}
		}
		if (skipBecauseOutside)
			continue;

		// project this edge segment to marker plane
		vector<Point2d> projPixels(edgeMap->segments[i].noPixels);
		vector<double> sampleErrors(36, INFINITY);
		vector<double> pixelErrors(edgeMap->segments[i].noPixels, INFINITY);
		for (int j = 0; j < edgeMap->segments[i].noPixels; j++)
		{
			Mat edgePix = (cv::Mat_<double>(3, 1) << edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r, 1);
			Mat projEdgePix = Hinv * edgePix;

			projPixels[j] = Point2d(projEdgePix.at<double>(0) / projEdgePix.at<double>(2), projEdgePix.at<double>(1) / projEdgePix.at<double>(2));
			for (int sampleInd = 0; sampleInd < 36; sampleInd++)
			{
				double dist = sqrt((projPixels[j].x - samplePoints[sampleInd].x) * (projPixels[j].x - samplePoints[sampleInd].x) + (projPixels[j].y - samplePoints[sampleInd].y) * (projPixels[j].y - samplePoints[sampleInd].y));

				if (dist < sampleErrors[sampleInd])
					sampleErrors[sampleInd] = dist;

				if (dist < pixelErrors[j])
					pixelErrors[j] = dist;
			}
		}

		// skip if at least one projected pixel is far away from the expected circle
		bool atLeastOneBadPixel = false;
		for (int j = 0; j < pixelErrors.size(); j++)
		{
			if (pixelErrors[j] > 0.1)
			{
				atLeastOneBadPixel = true;
				break;
			}
		}
		if (atLeastOneBadPixel)
			continue;

		double errorSum = std::accumulate(sampleErrors.begin(), sampleErrors.end(), (double)0);

		if ((errorSum < lowestError) && (errorSum < maxError))
		{
			lowestError = errorSum;
			chosenEdgeSegment = i;
		}
	}

	if (chosenEdgeSegment == -1)
		return false;

	accPixs = vector<pix>(edgeMap->segments[chosenEdgeSegment].noPixels);

	for (int i = 0; i < accPixs.size(); i++)
	{
		pix p;
		p.x = edgeMap->segments[chosenEdgeSegment].pixels[i].c;
		p.y = edgeMap->segments[chosenEdgeSegment].pixels[i].r;
		accPixs[i] = p;
	}

	borderEllipse = customEllipse(accPixs.data(), accPixs.size());

	// record the accumulated arc pixels real quick
	// note that there may be multiple markers, hence we push_back
	if (keepPaperTrail)
	{
		for (int i = 0; i < accPixs.size(); i++)
			arcPixels.push_back(Point2d(accPixs[i].x, accPixs[i].y));
	}

	// get the ellipse coefficients
	double coeffs[6];
	borderEllipse.GetCoefficients(coeffs);
	cand.C = Mat(3, 3, CV_64FC1);
	cand.C.at<double>(0, 0) = coeffs[0];
	cand.C.at<double>(0, 1) = cand.C.at<double>(1, 0) = -coeffs[1] / 2;
	cand.C.at<double>(1, 1) = coeffs[2];
	cand.C.at<double>(0, 2) = cand.C.at<double>(2, 0) = coeffs[3] / 2;
	cand.C.at<double>(1, 2) = cand.C.at<double>(2, 1) = -coeffs[4] / 2;
	cand.C.at<double>(2, 2) = coeffs[5];







	/*
	cand.center = Point2d(borderEllipse.GetCenterX(), borderEllipse.GetCenterY());
	return true;
	*/

	



	LMellipse = cand.C;

	cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
	cv::Ptr<cv::MinProblemSolver::Function> ptr_F = cv::makePtr<Refine>();
	solver->setFunction(ptr_F);

	

	cv::Mat x(1, 9, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			x.at<double>(i + j * 3) = cand.H.at<double>(i, j);

	cv::Mat step(9, 1, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			step.at<double>(i + j * 3) = abs(0.001 * x.at<double>(i + j * 3));
	solver->setInitStep(step);


	double res = solver->minimize(x);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cand.H.at<double>(i, j) = x.at<double>(i + j * 3);

	cand.center = projectPoint(Point2d(0.5, 0.5), cand.H);
	cand.corners[0] = projectPoint(Point2d(0, 0), cand.H);
	cand.corners[1] = projectPoint(Point2d(1, 0), cand.H);
	cand.corners[2] = projectPoint(Point2d(1, 1), cand.H);
	cand.corners[3] = projectPoint(Point2d(0, 1), cand.H);

	

	return true;
}

void EDMarkers::correctLineDirection(LineSegment& ls)
{
	// if invert == 0, the line is longer in x-axis
	// if invert == 1, the line is longer in y-axis
	// the number of pixels along the longer axis is the number of pixels to be sampled (sampling may be done sparsely if it's a bottleneck)
	int pointsToSample;
	int minX, maxX, minY, maxY;
	if (ls.invert == 0)
	{
		minX = (int)min(ls.sx, ls.ex);
		maxX = (int)(max(ls.sx, ls.ex) + 0.5);
		pointsToSample = maxX - minX + 1;
	}
	else
	{
		minY = (int)min(ls.sy, ls.ey);
		maxY = (int)(max(ls.sy, ls.ey) + 0.5);
		pointsToSample = maxY - minY + 1;
	}

	// line's left and right sides are sampled with an offset
	double offset = 1;

	// create vectors to hold the pixels to be sampled
	// right holds the right-hand side pixels when going from start to end, vice versa

	// the sampling is done using nearest neighborhood, hence the int type
	// these are the pixels to be sampled for each side
	vector<Point2i> right(pointsToSample);
	vector<Point2i> left(pointsToSample);

	//these are the pixels on the line
	double neutX, neutY;

	if (ls.invert == 0) // if the line is horizontal
	{
		if (ls.sx < ls.ex) // if the line goes left to right
		{
			for (int i = 0; i < pointsToSample; i++)
			{
				neutX = minX + i;
				neutY = ls.b * neutX + ls.a;

				right[i] = Point2i((int)neutX, (int)round(neutY + offset));
				left[i] = Point2i((int)neutX, (int)round(neutY - offset));
			}
		}
		else // if the line goes right to left
		{
			for (int i = 0; i < pointsToSample; i++)
			{
				neutX = minX + i;
				neutY = ls.b * neutX + ls.a;

				right[i] = Point2i((int)neutX, (int)round(neutY - offset));
				left[i] = Point2i((int)neutX, (int)round(neutY + offset));
			}
		}
	}

	else // if the line is vertical
	{
		if (ls.sy < ls.ey) // if the line goes up to down
		{
			for (int i = 0; i < pointsToSample; i++)
			{
				neutY = minY + i;
				neutX = ls.b * neutY + ls.a;

				right[i] = Point2i((int)round(neutX - offset), (int)neutY);
				left[i] = Point2i((int)round(neutX + offset), (int)neutY);
			}
		}
		else // if the line goes down to up
		{
			for (int i = 0; i < pointsToSample; i++)
			{
				neutY = minY + i;
				neutX = ls.b * neutY + ls.a;

				right[i] = Point2i((int)round(neutX + offset), (int)neutY);
				left[i] = Point2i((int)round(neutX - offset), (int)neutY);
			}
		}
	}

	// read image and accumulate brightness levels from the pixels

	// first check if a pixel outside the borders is meant to be read
	// if so, use safe read
	minX = min(min(right[0].x, right.back().x), min(left[0].x, left.back().x));
	maxX = max(max(right[0].x, right.back().x), max(left[0].x, left.back().x));
	minY = min(min(right[0].y, right.back().y), min(left[0].y, left.back().y));
	maxY = max(max(right[0].y, right.back().y), max(left[0].y, left.back().y));

	bool useSafeRead = false;
	if ((minX < 0) || (maxX >= width) || (minY < 0) || (maxY >= height))
		useSafeRead = true;

	// now read each pixel and accumulate brightness levels
	unsigned int accRight = 0, accLeft = 0;

	if (useSafeRead)
	{
		for (int i = 0; i < pointsToSample; i++)
		{
			accRight += readPixelSafe(right[i]);
			accLeft += readPixelSafe(left[i]);
		}
	}
	else
	{
		for (int i = 0; i < pointsToSample; i++)
		{
			accRight += readPixelUnsafe(right[i]);
			accLeft += readPixelUnsafe(left[i]);
		}
	}

	// when going from start to end, right-hand side should have been darker => lesser accumulation
	// if this is not the case, replace sx-sy with ex-ey
	if (accLeft < accRight)
	{
		double t1 = ls.sx;
		double t2 = ls.sy;
		ls.sx = ls.ex;
		ls.sy = ls.ey;
		ls.ex = t1;
		ls.ey = t2;
	}
}

void EDMarkers::cleanUp()
{
	if (edLines != NULL)
		delete edLines;
	if (edgeMap != NULL)
		delete edgeMap;

	timings = vector<double>();
	cornerGroups = vector<vector<Corner>>();
	curvCornerInds = vector<vector<int>>();
	markers = vector<Marker>();

	roughMarkers = vector<Marker>();
	distMarkers = vector<Marker>();
	invalidMarkers = vector<Marker>();
	arcPixels = vector<Point2d>();
}

void EDMarkers::occlusionResistantDetection()
{
	// correct line directions by sampling the image
	// if this part is skipped, quads are generated from line segments regardless of their direction
	if (flagCorrectLineDirections)
	{
		for (int i = 0; i < edLines->noLines; i++)
			correctLineDirection(edLines->lines[i]);
	}

	// since this is occlusion resistan, all corners will be in the same cornerGroup
	vector<Corner> corners;

	for (int i = 0; i < edLines->noLines; i++)
	{
		for (int j = i + 1; j < edLines->noLines; j++)
		{
			LineSegment line1 = edLines->lines[i];
			LineSegment line2 = edLines->lines[j];

			if (flagCorrectLineDirections)
			{
				Point2d vec1start1end(line1.ex - line1.sx, line1.ey - line1.sy);
				Point2d vec1start2end(line2.ex - line1.sx, line2.ey - line1.sy);

				// change condition direction if quad is white inside
				if (crosProd(vec1start1end, vec1start2end) <= 0)
					continue;
			}

			Point2d inters = intersOfLineSegments(line1, line2);

			// don't form a corner if the intersection is on one of the lines (this wan't a problem in the non-occlusion resistant version)
			double minX, maxX;

			minX = min(line1.sx, line1.ex) + thresManhDist;
			maxX = max(line1.sx, line1.ex) - thresManhDist;
			if ((inters.x < maxX) && (inters.x > minX))
				continue;

			minX = min(line2.sx, line2.ex) + thresManhDist;
			maxX = max(line2.sx, line2.ex) - thresManhDist;
			if ((inters.x < maxX) && (inters.x > minX))
				continue;

			// checks if the corner is on the edge segment of each line
			if (flagCheckCornerDistToEdges)
			{
				bool onTheSegment = false;
				for (int edgePixInd = 0; edgePixInd < edgeMap->segments[line1.segmentNo].noPixels; edgePixInd++)
				{
					if (abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].c - inters.x) + abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].r - inters.y) < thresManhDist)
					{
						onTheSegment = true;
						break;
					}
				}
				if (!onTheSegment)
					continue;

				onTheSegment = false;
				for (int edgePixInd = 0; edgePixInd < edgeMap->segments[line2.segmentNo].noPixels; edgePixInd++)
				{
					if (abs(edgeMap->segments[line2.segmentNo].pixels[edgePixInd].c - inters.x) + abs(edgeMap->segments[line2.segmentNo].pixels[edgePixInd].r - inters.y) < thresManhDist)
					{
						onTheSegment = true;
						break;
					}
				}

				if (!onTheSegment)
					continue;
			}

			corners.push_back(Corner(inters, line1, line2));
		}
	}

	cornerGroups.push_back(corners);

	// create quads using 2 opposite corners

	vector<Marker> markerCandidates;

	for (int i = 0; i < corners.size(); i++)
	{
		for (int j = i + 1; j < corners.size(); j++)
		{
			Corner c1 = corners[i];
			Corner c3 = corners[j];

			if (!checkIfTwoCornersAreOpposite(c1, c3))
				continue;

			// estimate c2 and c4 using c1 and c3
			Corner c2, c4;
			// there is two combinations when forming c2 and c4
			// we try the first one, check if it works. if not, use the second one.
			c2 = Corner(intersOfLineSegments(c1.l1, c3.l1), c1.l1, c3.l1);
			c4 = Corner(intersOfLineSegments(c1.l2, c3.l2), c1.l2, c3.l2);
			if (!checkIfFourCornersAreOpposite(c1, c2, c3, c4))
			{
				c2 = Corner(intersOfLineSegments(c1.l1, c3.l2), c1.l1, c3.l2);
				c4 = Corner(intersOfLineSegments(c1.l2, c3.l1), c1.l2, c3.l1);
			}

			// order corners in clockwise
			Point2d vec13(c3.loc.x - c1.loc.x, c3.loc.y - c1.loc.y);
			Point2d vec12(c2.loc.x - c1.loc.x, c2.loc.y - c1.loc.y);

			if (crosProd(vec13, vec12) > 0)
			{
				Corner temp = c2;
				c2 = c4;
				c4 = temp;
			}

			Marker cand(c1.loc, c2.loc, c3.loc, c4.loc);

			if (flagCheckProjectiveDistortion && (cand.calcProjDist() > thresProjDist))
				continue;

			if (decodeCandidate(cand))
			{
				markerCandidates.push_back(cand);
				// stop searching for more markers on this segment
				break;
			}
			else if (keepPaperTrail)
				invalidMarkers.push_back(cand);
		}
	}

	// each marker produces two candidates, one of which we will eliminate later on
	// if the eliminated candidates interchange at each frame, this will cause jitter
	// to reduce this jitter, corners of two candidates are averaged, which is far from ideal

	// eliminate the repeated markers
	// this assumes that there will not be multiple markers with the same ID on the scene

	vector<Marker> singleMarkerCandidates;

	for (int i = 0; i < markerCandidates.size(); i++)
	{
		bool addThisMarker = true;;

		for (int j = 0; j < singleMarkerCandidates.size(); j++)
		{
			if (markerCandidates[i].id == singleMarkerCandidates[j].id)
			{
				bool sameMarker = true;
				for (int k = 0; k < 4; k++)
				{
					if (abs(markerCandidates[i].corners[k].x - singleMarkerCandidates[j].corners[k].x) + abs(markerCandidates[i].corners[k].y - singleMarkerCandidates[j].corners[k].y) > thresManhDist)
					{
						sameMarker = false;
						break;
					}
				}

				if (sameMarker)
				{
					for (int k = 0; k < 4; k++)
					{
						singleMarkerCandidates[j].corners[k].x = (singleMarkerCandidates[j].corners[k].x + markerCandidates[i].corners[k].x) / 2;
						singleMarkerCandidates[j].corners[k].y = (singleMarkerCandidates[j].corners[k].y + markerCandidates[i].corners[k].y) / 2;
					}
					addThisMarker = false;
					break;
				}

			}
		}
		if (addThisMarker)
			singleMarkerCandidates.push_back(markerCandidates[i]);
	}

	for (int i = 0; i < singleMarkerCandidates.size(); i++)
	{
		if (flagRefinePose)
		{
			if (keepPaperTrail)
				roughMarkers.push_back(singleMarkerCandidates[i]);
			refinePoseEstimation(singleMarkerCandidates[i]);
		}

		markers.push_back((Marker)singleMarkerCandidates[i]);
	}
}

EDMarkers::EDMarkers(int libraryHD, int errorCorrection)
{
	initialize(libraryHD, errorCorrection);
}


unsigned char EDMarkers::readPixelUnsafe(const Point2i& p)
{
	return image.ptr<unsigned char>(p.y)[p.x];
}

unsigned char EDMarkers::readPixelSafe(const Point2i& p)
{
	if ((p.x >= 0) && (p.x < width) && (p.y >= 0) && (p.y < height))
		return image.ptr<unsigned char>(p.y)[p.x];
	else
		return (unsigned char)128; // if the point to be read is outside image boundaries, return 128
}

unsigned char EDMarkers::readPixelSafeBilinear(const Point2d& p)
{
	if ((p.x >= 0) && (p.x <= width - 1) && (p.y >= 0) && (p.y <= height - 1))
	{
		int x1 = floor(p.x);
		int x2 = ceil(p.x);
		int y1 = floor(p.y);
		int y2 = ceil(p.y);

		double dist1 = sqrt((x1 - p.x) * (x1 - p.x) + (y1 - p.y) * (y1 - p.y));
		double dist2 = sqrt((x1 - p.x) * (x1 - p.x) + (y2 - p.y) * (y2 - p.y));
		double dist3 = sqrt((x2 - p.x) * (x2 - p.x) + (y1 - p.y) * (y1 - p.y));
		double dist4 = sqrt((x2 - p.x) * (x2 - p.x) + (y2 - p.y) * (y2 - p.y));
		double totDist = dist1 + dist2 + dist3 + dist4;

		double totalRead = 0;
		unsigned char reading;

		reading = image.ptr<unsigned char>(y1)[x1];;
		totalRead += reading * dist1;

		reading = image.ptr<unsigned char>(y2)[x1];;
		totalRead += reading * dist2;

		reading = image.ptr<unsigned char>(y1)[x2];
		totalRead += reading * dist3;

		reading = image.ptr<unsigned char>(y2)[x2];
		totalRead += reading * dist4;

		return totalRead / totDist;
	}
	else
		return (unsigned char)128; // if the point to be read is outside image boundaries, return 128
}



void EDMarkers::colorAPixel(Mat& img, int x, int y, cv::Scalar color, int dotWidth)
{
	for (int i = y - dotWidth; i < y + dotWidth + 1; i++)
	{
		for (int j = x - dotWidth; j < x + dotWidth + 1; j++)
		{
			if ((i >= 0) && (i < img.rows) && (j >= 0) && (j < img.cols))
			{
				img.at<cv::Vec3b>(i, j)[0] = color.val[0];
				img.at<cv::Vec3b>(i, j)[1] = color.val[1];
				img.at<cv::Vec3b>(i, j)[2] = color.val[2];
			}
		}
	}
}

void EDMarkers::drawEdgeMap(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	int dotWidth = 1;

	for (int i = 0; i < edgeMap->noSegments; i++)
	{
		for (int j = 0; j < edgeMap->segments[i].noPixels; j++)
		{
			colorAPixel(bgrMat, edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r, colors[i % colors.size()], dotWidth);
		}
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "1 edges " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawLines(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	int currSegment = -1;
	for (int i = 0; i < edLines->noLines; i++)
	{
		if (edLines->lines[i].segmentNo != currSegment)
			currSegment = edLines->lines[i].segmentNo;

		cv::line(bgrMat, cv::Point(edLines->lines[i].sx, edLines->lines[i].sy), cv::Point(edLines->lines[i].ex, edLines->lines[i].ey), colors[currSegment % colors.size()], 2, CV_AA);
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "2 lines " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawCorners(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	for (int i = 0; i < cornerGroups.size(); i++)
	{
		for (int j = 0; j < cornerGroups[i].size(); j++)
		{
			cv::circle(bgrMat, cv::Point(cornerGroups[i][j].loc.x, cornerGroups[i][j].loc.y), 3, colors[i % colors.size()], 1, CV_AA);
		}
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "3 corners " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawMarkers(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	for (int i = 0; i < markers.size(); i++)
	{
		cv::circle(bgrMat, cv::Point(markers[i].corners[0].x, markers[i].corners[0].y), 5, cv::Scalar(0, 255, 0), 1, CV_AA);
		for (int j = 0; j < 4; j++)
			cv::line(bgrMat, cv::Point(markers[i].corners[j].x, markers[i].corners[j].y), cv::Point(markers[i].corners[(j + 1) % 4].x, markers[i].corners[(j + 1) % 4].y), cv::Scalar(50, 255, 50), 2, CV_AA);

		cv::circle(bgrMat, cv::Point(markers[i].center.x, markers[i].center.y), 5, cv::Scalar(0, 255, 0), 1, CV_AA);

		cv::putText(bgrMat, std::to_string(markers[i].id), cv::Point((markers[i].corners[0].x + markers[i].corners[2].x) / 2, (markers[i].corners[0].y + markers[i].corners[2].y) / 2), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 50, 255));
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "4 markers " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawInvalidMarkers(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	for (int i = 0; i < invalidMarkers.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cv::line(bgrMat, cv::Point(invalidMarkers[i].corners[j].x, invalidMarkers[i].corners[j].y), cv::Point(invalidMarkers[i].corners[(j + 1) % 4].x, invalidMarkers[i].corners[(j + 1) % 4].y), cv::Scalar(50, 255, 50), 2, CV_AA);
		}
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "5 invalid_markers " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawDistMarkers(const string& path, const string& name)
{
	if (!flagCheckProjectiveDistortion)
		return;

	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	for (int i = 0; i < distMarkers.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cv::line(bgrMat, cv::Point(distMarkers[i].corners[j].x, distMarkers[i].corners[j].y), cv::Point(distMarkers[i].corners[(j + 1) % 4].x, distMarkers[i].corners[(j + 1) % 4].y), cv::Scalar(50, 255, 50), 2, CV_AA);
		}
		cv::putText(bgrMat, std::to_string(distMarkers[i].projDist), cv::Point((distMarkers[i].corners[0].x + distMarkers[i].corners[2].x) / 2, (distMarkers[i].corners[0].y + distMarkers[i].corners[2].y) / 2), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 50, 255));
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "6 dist_markers " + name + ".png", bgrMat, compressionParams);
}


void EDMarkers::drawRoughMarkers(const string& path, const string& name)
{
	if (!flagRefinePose)
		return;

	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	for (int i = 0; i < markers.size(); i++)
	{
		cv::circle(bgrMat, cv::Point(roughMarkers[i].corners[0].x, roughMarkers[i].corners[0].y), 5, cv::Scalar(0, 255, 0), 1, CV_AA);
		for (int j = 0; j < 4; j++)
		{
			cv::line(bgrMat, cv::Point(roughMarkers[i].corners[j].x, roughMarkers[i].corners[j].y), cv::Point(roughMarkers[i].corners[(j + 1) % 4].x, roughMarkers[i].corners[(j + 1) % 4].y), cv::Scalar(50, 255, 50), 2, CV_AA);
		}
		cv::circle(bgrMat, cv::Point(roughMarkers[i].center.x, roughMarkers[i].center.y), 5, cv::Scalar(0, 255, 0), 1, CV_AA);

		cv::putText(bgrMat, std::to_string(roughMarkers[i].id), cv::Point((roughMarkers[i].corners[0].x + roughMarkers[i].corners[2].x) / 2, (roughMarkers[i].corners[0].y + roughMarkers[i].corners[2].y) / 2), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 50, 255));
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "7 rough_markers " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawArcs(const string& path, const string& name)
{
	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	int dotWidthEdge = 0;
	int dotWidthArc = 2;

	// draw the edges
	for (int i = 0; i < edgeMap->noSegments; i++)
	{
		for (int j = 0; j < edgeMap->segments[i].noPixels; j++)
		{
			colorAPixel(bgrMat, edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r, colors[i % colors.size()], dotWidthEdge);
		}
	}

	// mark the curvature corners
	for (int i = 0; i < curvCornerInds.size(); i++)
	{
		for (int j = 0; j < curvCornerInds[i].size(); j++)
			cv::circle(bgrMat, cv::Point(edgeMap->segments[i].pixels[curvCornerInds[i][j]].c, edgeMap->segments[i].pixels[curvCornerInds[i][j]].r), 3, colors[i % colors.size()], 1, CV_AA);
	}

	// redraw the chosen edge segments with green
	for (int i = 0; i < arcPixels.size(); i++)
	{
		colorAPixel(bgrMat, arcPixels[i].x, arcPixels[i].y, cv::Scalar(0, 255, 0), dotWidthArc);
	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "8 arcs " + name + ".png", bgrMat, compressionParams);
}

void EDMarkers::drawEllipses(const string& path, const string& name)
{
	if (!flagRefinePose)
		return;

	Mat greyMat = image.clone();
	Mat bgrMat;
	cv::cvtColor(greyMat, bgrMat, CV_GRAY2BGR);

	int dotWidth = 1;

	// using my very own ellipse drawing algorithm
	// original do not steal (TM)

	for (int i = 0; i < markers.size(); i++)
	{
		if (markers[i].C.data == NULL)
			continue;

		for (int y = 0; y < height; y++)
		{
			vector<double> xOfPointsOnConic;

			double a = markers[i].C.at<double>(0, 0);
			double b = markers[i].C.at<double>(0, 1) * 2 * y + markers[i].C.at<double>(0, 2) * 2;
			double c = markers[i].C.at<double>(1, 1) * y * y + markers[i].C.at<double>(1, 2) * 2 * y + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				xOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				xOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				xOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < xOfPointsOnConic.size(); j++)
			{
				xOfPointsOnConic[j] = round(xOfPointsOnConic[j]);

				if ((xOfPointsOnConic[j] < 0) || (xOfPointsOnConic[j] >= width))
					continue;

				colorAPixel(bgrMat, xOfPointsOnConic[j], y, cv::Scalar(0, 255, 0), dotWidth);
			}
		}

		for (int x = 0; x < width; x++)
		{
			vector<double> yOfPointsOnConic;

			double a = markers[i].C.at<double>(1, 1);
			double b = markers[i].C.at<double>(0, 1) * 2 * x + markers[i].C.at<double>(1, 2) * 2;
			double c = markers[i].C.at<double>(0, 0) * x * x + markers[i].C.at<double>(0, 2) * 2 * x + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				yOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				yOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				yOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < yOfPointsOnConic.size(); j++)
			{
				yOfPointsOnConic[j] = round(yOfPointsOnConic[j]);

				if ((yOfPointsOnConic[j] < 0) || (yOfPointsOnConic[j] >= height))
					continue;

				colorAPixel(bgrMat, x, yOfPointsOnConic[j], cv::Scalar(0, 255, 0), dotWidth);
			}
		}

	}
	vector<int> compressionParams = { CV_IMWRITE_PNG_COMPRESSION, 0 };
	cv::imwrite(path + "9 ellipses " + name + ".png", bgrMat, compressionParams);
}


void EDMarkers::writeResults(const string& path, const string& name)
{
	using std::endl;
	std::ofstream textFile(path + "textlog " + name + ".txt");

	textFile << "Markers" << endl;
	textFile << "# of markers" << endl;
	textFile << markers.size() << endl;
	for (int i = 0; i < markers.size(); i++)
	{
		textFile << "Marker ID" << endl;
		textFile << markers[i].id << endl;
		textFile << "Marker center" << endl;
		textFile << markers[i].center.x << " " << markers[i].center.y << endl;
		textFile << "Marker corners" << endl;
		for (int j = 0; j < 4; j++)
			textFile << markers[i].corners[j].x << " " << markers[i].corners[j].y << endl;
	}

	textFile << "Invalid Markers" << endl;
	textFile << "# of invalid markers" << endl;
	textFile << invalidMarkers.size() << endl;

	if (flagCheckProjectiveDistortion)
	{
		textFile << "Distorted Markers" << endl;
		textFile << "# of distorted markers" << endl;
		textFile << distMarkers.size() << endl;
		textFile << "Projective distortions" << endl;
		for (int i = 0; i < distMarkers.size(); i++)
			textFile << distMarkers[i].projDist << endl;
	}

	if (flagRefinePose)
	{
		textFile << "Rough markers" << endl;
		textFile << "# of markers" << endl;
		textFile << roughMarkers.size() << endl;
		for (int i = 0; i < markers.size(); i++)
		{
			textFile << "Marker ID" << endl;
			textFile << roughMarkers[i].id << endl;
			textFile << "Marker center" << endl;
			textFile << roughMarkers[i].center.x << " " << roughMarkers[i].center.y << endl;
			textFile << "Marker corners" << endl;
			for (int j = 0; j < 4; j++)
				textFile << roughMarkers[i].corners[j].x << " " << roughMarkers[i].corners[j].y << endl;
		}
	}

	textFile.close();
}



void EDMarkers::detectMarkers(Mat image, bool debug)
{
	// if not initialized before, initialize with default values (15 HD, 7-bit error correction)
	if (!initialized)
		initialize(15, 7);

	cleanUp();

	keepPaperTrail = debug;
	this->image = image;
	width = image.size().width;
	height = image.size().height;

	Timer totalTimer;
	totalTimer.Start();


	// EDLines doesn't look for line segments on edge segments which are not loops if flagOnlyUseEdgeSegmentLoops
	// An edge segment is a loop if the Manhattan distance between its start and end point is smaller than thresManhDist
	edLines = DetectLinesByEDPF(edgeMap, image.data, image.size().width, image.size().height, flagOnlyUseEdgeSegmentLoops, thresManhDist);

	// Non-fully occlusion resistant is the default mode
	if (!flagFullyOcclusionResistant)
	{
		// put the line segments from each edge segment to a seperate group if there are more than 4
		vector<vector<int>> lineGroups;

		int noOfLinesInCurrentSegment = 0;
		int currentSegment = edLines->lines[0].segmentNo;

		for (int i = 0; i < edLines->noLines; i++)
		{
			if (edLines->lines[i].segmentNo == currentSegment)
			{
				noOfLinesInCurrentSegment++;
				continue;
			}

			if (noOfLinesInCurrentSegment >= 4)
			{
				lineGroups.push_back(vector<int>());
				for (int j = 0; j < noOfLinesInCurrentSegment; j++)
					lineGroups.back().push_back(i - noOfLinesInCurrentSegment + j);
			}

			currentSegment = edLines->lines[i].segmentNo;
			noOfLinesInCurrentSegment = 1;
		}

		if (noOfLinesInCurrentSegment >= 4)
		{
			lineGroups.push_back(vector<int>());
			for (int j = 0; j < noOfLinesInCurrentSegment; j++)
				lineGroups.back().push_back(edLines->noLines - noOfLinesInCurrentSegment + j);
		}

		// correct line directions by sampling the image
		// if this part is skipped, quads are generated from line segments regardless of their direction
		if (flagCorrectLineDirections)
		{
			for (int i = 0; i < lineGroups.size(); i++)
			{
				for (int j = 0; j < lineGroups[i].size(); j++)
					correctLineDirection(edLines->lines[lineGroups[i][j]]);

				// ensure this order: line1.start->line1.end->line2.start->line2.end->line3.start...
				LineSegment line1 = edLines->lines[lineGroups[i][0]];
				LineSegment line2 = edLines->lines[lineGroups[i][1]];

				Point2d inters = intersOfLineSegments(line1, line2);

				if (abs(line1.sx - inters.x) + abs(line1.sy - inters.y) < abs(line1.ex - inters.x) + abs(line1.ey - inters.y))
					std::reverse(lineGroups[i].begin(), lineGroups[i].end());
			}
		}

		// create corner groups using the line groups
		for (int lineGroupInd = 0; lineGroupInd < lineGroups.size(); lineGroupInd++)
		{
			bool createdNewCornerGroup = false;
			for (int lineInd = 0; lineInd < lineGroups[lineGroupInd].size(); lineInd++)
			{
				int lineIndNext = (lineInd + 1) % (lineGroups[lineGroupInd].size());

				LineSegment line1 = edLines->lines[lineGroups[lineGroupInd][lineInd]];
				LineSegment line2 = edLines->lines[lineGroups[lineGroupInd][lineIndNext]];

				// we can't check if the quad is darker inside if we did not correct line directions
				if (flagCorrectLineDirections)
				{
					Point2d vec1start1end(line1.ex - line1.sx, line1.ey - line1.sy);
					Point2d vec1start2end(line2.ex - line1.sx, line2.ey - line1.sy);

					// the below condition direction looks for quads that are darker inside
					// if you are looking for quads that are lighter inside, simply change the condition direction
					if (crosProd(vec1start1end, vec1start2end) <= 0)
						continue;
				}

				Point2d inters = intersOfLineSegments(line1, line2);

				// by checking the corner distance to edge segment, we make sure that corners are not coming from nonlinear features
				if (flagCheckCornerDistToEdges)
				{
					bool onTheSegment = false;
					for (int edgePixInd = 0; edgePixInd < edgeMap->segments[line1.segmentNo].noPixels; edgePixInd++)
					{
						if (abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].c - inters.x) + abs(edgeMap->segments[line1.segmentNo].pixels[edgePixInd].r - inters.y) < thresManhDist)
						{
							onTheSegment = true;
							break;
						}
					}

					if (!onTheSegment)
						continue;
				}

				if (!createdNewCornerGroup)
				{
					cornerGroups.push_back(vector<Corner>());
					createdNewCornerGroup = true;
				}
				cornerGroups.back().push_back(Corner(inters, line1, line2));
			}
		}


		// create quads using corner groups
		for (int cornerGroupIndex = 0; cornerGroupIndex < cornerGroups.size(); cornerGroupIndex++)
		{
			// assumed that at least 3 corners are needed. actually, we need at least two opposite corners.
			// however, it is assumed that there is an additional corner between opposite corners, hence the need for 3 corners
			if (cornerGroups[cornerGroupIndex].size() < 3)
				continue;

			vector<Corner> currCornerGroup = cornerGroups[cornerGroupIndex];

			for (unsigned int cornerInd = 0; cornerInd < currCornerGroup.size(); cornerInd++)
			{
				int i1 = cornerInd, i2 = (i1 + 1) % currCornerGroup.size(), i3 = (i1 + 2) % currCornerGroup.size(), i4 = (i1 + 3) % currCornerGroup.size();

				Corner c1 = currCornerGroup[i1], c2 = currCornerGroup[i2], c3 = currCornerGroup[i3], c4 = currCornerGroup[i4];

				// if there are only 3 corners, replace the 4th corner with an invalid one
				if (i1 == i4)
					c4 = Corner(Point2d(INFINITY, INFINITY), LineSegment(), LineSegment());

				if (!checkFourCorners(c1, c2, c3, c4, thresManhDist))
					continue;

				Marker cand(c1.loc, c2.loc, c3.loc, c4.loc);

				// eliminate if projective distortion is larger than threshold
				if (flagCheckProjectiveDistortion && (cand.calcProjDist() > thresProjDist))
				{
					if (keepPaperTrail)
						distMarkers.push_back(cand);
					continue;
				}

				if (decodeCandidate(cand))
				{
					if (flagRefinePose)
					{
						if (keepPaperTrail)
							roughMarkers.push_back(cand);
						refinePoseEstimation(cand);
					}

					markers.push_back((Marker)cand);
					// stop searching for more markers on this segment
					break;
				}
				else if (keepPaperTrail)
				{
					invalidMarkers.push_back(cand);
				}

			}
		}
	}
	// made this a separate function because I don't want it to crowd the constructor
	else
		occlusionResistantDetection();

	totalTimer.Stop();
	timings.push_back(totalTimer.ElapsedTime());
}

EDMarkers::~EDMarkers()
{
	if (edLines != NULL)
		delete edLines;
	if (edgeMap != NULL)
		delete edgeMap;
}

void EDMarkers::initialize(int libraryHD, int errorCorrection)
{
	// set the marker library
	errCorr = errorCorrection;
	decoder = Decoder(libraryHD);

	// initialize code locations if they are not already initialized
	if (codeLocs.size() == 0)
		fillCodeLocations();

	initialized = true;
}

void EDMarkers::dumpLog(const string& path, const string& name)
{
	if (colors.size() == 0)
		colors = generateColorPalette();

	drawEdgeMap(path, name);
	drawLines(path, name);
	drawCorners(path, name);
	drawMarkers(path, name);
	drawInvalidMarkers(path, name);

	if (flagCheckProjectiveDistortion)
		drawDistMarkers(path, name);

	if (flagRefinePose)
	{
		drawRoughMarkers(path, name);
		drawArcs(path, name);
		drawEllipses(path, name);
	}

	writeResults(path, name);
}

vector<Marker> EDMarkers::getMarkers()
{
	return markers;
}

vector<Marker> EDMarkers::getRoughMarkers()
{
	return roughMarkers;
}

vector<double> EDMarkers::getTimings()
{
	return timings;
}

EDLines* EDMarkers::getLines()
{
	return edLines;
}

EdgeMap* EDMarkers::getEdgeMap()
{
	return edgeMap;
}

void EDMarkers::setRefinementFlag(bool refine)
{
	flagRefinePose = refine;
}



int LMFunc(int m, int n, double *p, double *deviates, double **derivs, void *privateData)
{
	// get the current homography matrix to a Mat
	Mat H, HT, Hi;
	H = Mat(3, 3, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			H.at<double>(i, j) = p[i + j * 3];
	}


	cv::transpose(H, HT);
	Hi = H.inv();

	// project the detected ellipse using the current homography matrix
	Mat projCircle = HT * LMellipse * H;

	vector<double> projEllipseCoeff(6);
	projEllipseCoeff[0] = projCircle.at<double>(0, 0);
	projEllipseCoeff[1] = -projCircle.at<double>(0, 1) * 2;
	projEllipseCoeff[2] = projCircle.at<double>(1, 1);
	projEllipseCoeff[3] = projCircle.at<double>(0, 2) * 2;
	projEllipseCoeff[4] = -projCircle.at<double>(1, 2) * 2;
	projEllipseCoeff[5] = projCircle.at<double>(2, 2);

	// construct the ellipse using coefficients (to calculate semi major/minor axes)
	customEllipse projEllipse(projEllipseCoeff.data());

	double factor = 2;

	// calculate the deviations from the border circle
	deviates[0] = abs(projEllipse.GetSemiMajorAxis() - 0.4) * factor;
	deviates[1] = deviates[0];
	deviates[2] = abs(projEllipse.GetSemiMinorAxis() - 0.4) * factor;
	deviates[3] = deviates[2];
	deviates[4] = abs(projEllipse.GetCenterX() - 0.5) * factor;
	deviates[5] = deviates[4];
	deviates[6] = abs(projEllipse.GetCenterY() - 0.5) * factor;
	deviates[7] = deviates[6];

	// backproject corners to the marker plane and use them as additional constraints
	double cornersX[4] = { 0, 1, 1, 0 };
	double cornersY[4] = { 0, 0, 1, 1 };

	for (int i = 0; i < 4; i++)
	{
		Mat projCorner = Hi * LMcornerPixels[i];
		double dx = cornersX[i] - projCorner.at<double>(0) / projCorner.at<double>(2);
		double dy = cornersY[i] - projCorner.at<double>(1) / projCorner.at<double>(2);

		deviates[8 + i] = dx * dx + dy * dy;
	}

	return 0;
}