#include "CurvatureCornerDetection.h"

using std::vector;

double cornerThresh = 9; // reduced this from 9 to detect more corners
double lineFitErrThr = 1.5;
double maxLineLen = 7;
double angleThresh = 155;

vector<int> DetectAndValidateCurvatureCorners(const EdgeSegment &edgeSeg)
{
	Pixel* pArr = edgeSeg.pixels;
	int noPix = edgeSeg.noPixels;

	if (noPix < 12)
		return vector<int>();

	vector<double> xArr(noPix);
	vector<double> yArr(noPix);
	vector<double> smoothX(noPix);
	vector<double> smoothY(noPix);
	vector<double> firstDX(noPix);
	vector<double> firstDY(noPix);
	vector<double> seconDX(noPix);
	vector<double> seconDY(noPix);
	vector<double> curvature = vector<double>(noPix);

	vector<int> cornerInd;


	for (int i = 0; i < noPix; i++)
	{
		xArr[i] = pArr[i].c;
		yArr[i] = pArr[i].r;
	}

	//Smoothing
	SmoothSegment(smoothX.data(), xArr.data(), noPix);
	SmoothSegment(smoothY.data(), yArr.data(), noPix);

	DerivateSegment(firstDX.data(), smoothX.data(), noPix);
	DerivateSegment(firstDY.data(), smoothY.data(), noPix);

	DerivateSegment(seconDX.data(), firstDX.data(), noPix);
	DerivateSegment(seconDY.data(), firstDY.data(), noPix);

	//Curvature Computation (x100)
	for (int i = 0; i < noPix; i++)
	{
		curvature[i] = 100 * abs((firstDX[i] * seconDY[i] - seconDX[i] * firstDY[i]) /
			pow((firstDX[i] * firstDX[i] + firstDY[i] * firstDY[i]), 1.5));
	}

	//DETECT CORNERS	
	cornerInd.push_back(0);

	int start = 5;
	for (int i = start; i < noPix - start - 1; i++)
	{
		if (curvature[i] < cornerThresh) 
			continue;

		if (curvature[i] > curvature[i - 1] && curvature[i] >= curvature[i + 1])
		{
			//Validation
			Pixel P1, P2, P3;
			P2 = pArr[i];
			int invert, prCount, nxCount, maxAngIdx = i;

			double pErr = 0, nErr = 0;
			double maxAngle = curvature[i];
			double a, b, errThr = lineFitErrThr;

			for (prCount = 4; prCount < maxLineLen; prCount++)
			{
				if (maxAngIdx - prCount <= 0) break;

				LineFit(xArr.data(), yArr.data(), maxAngIdx - prCount, prCount, a, b, pErr, invert);

				if (pErr > errThr)
				{
					prCount--;
					break;
				}
			}
			if (prCount < 5) continue;
			P1 = pArr[maxAngIdx - prCount];

			for (nxCount = 4; nxCount < maxLineLen; nxCount++)
			{
				if (maxAngIdx + nxCount >= noPix - 1) break;

				LineFit(xArr.data(), yArr.data(), maxAngIdx, nxCount, a, b, nErr, invert);

				if (nErr > errThr)
				{
					nxCount--;
					break;
				}
			}
			if (nxCount < 5) continue;
			P3 = pArr[maxAngIdx + nxCount];

			//Compute angle between lines
			Pixel V1, V2;
			V1.c = P2.c - P1.c, V1.r = P2.r - P1.r;
			V2.c = P2.c - P3.c, V2.r = P2.r - P3.r;

			double norm1 = V1.c * V1.c + V1.r * V1.r;
			double norm2 = V2.c * V2.c + V2.r * V2.r;
			double cosTheta = (V1.c * V2.c + V1.r * V2.r) / (sqrt(norm1) * sqrt(norm2));

			double theta = acos(cosTheta) * 180.0 / 3.14159;

			if (theta < angleThresh && theta == theta) //isnan(theta)?
				cornerInd.push_back(i);
		}
	}

	cornerInd.push_back(noPix - 1);  //Add last pixel as the first corner
	return cornerInd;
}

inline void SmoothSegment(double *dst, double *src, int len)
{
	//for (int i = 3; i < len - 3; i++)
	//{
	//	//1       2       2       3       2       2       1  
	//	dst[i] =
	//		(src[i - 3] + 2 * src[i - 2] +
	//		2 * src[i - 1] + 3 * src[i] + 2 * src[i + 1] +
	//		2 * src[i + 2] + src[i + 3]) / 13;		
	//}

	for (int i = 3; i < len - 3; i++)
	{
		//1       2       3       5       3       2       1  
		dst[i] =
			(src[i - 3] + 2 * src[i - 2] + 3 * src[i - 1] +
			4 * src[i] +
			3 * src[i + 1] + 2 * src[i + 2] + src[i + 3]) / 17;
	}
	//fill the first & last part of the smoothangles
	for (int i = 0; i < 3; i++) dst[i] = dst[3];
	for (int i = len - 3; i < len; i++) dst[i] = dst[len - 4];
}

inline void DerivateSegment(double *dst, double *src, int len)
{
	for (int i = 1; i < len - 1; i++)
	{
		dst[i] = abs(src[i - 1] - src[i + 1]);

		//if(dst[i] < 0) dst[i] = 0;
	}
	//fill the first & last part of the derivAngles
	dst[0] = dst[1];
	dst[len - 1] = dst[len - 2];
}

inline void LineFit(double *x, double *y, int start, int count, double &a, double &b, double &e, int &invert)
{
	if (count < 2) return;

	double S = count, Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
	for (int i = start; i < start + count; i++)
	{
		Sx += x[i];
		Sy += y[i];
	} //end-for

	double mx = Sx / count;
	double my = Sy / count;

	double dx = 0.0;
	double dy = 0.0;
	for (int i = start; i < start + count; i++)
	{
		dx += (x[i] - mx) * (x[i] - mx);
		dy += (y[i] - my) * (y[i] - my);
	} //end-for

	if (dx < dy)
	{
		// Vertical line. Swap x & y, Sx & Sy
		invert = 1;
		double* t = x;
		x = y;
		y = t;

		double d = Sx;
		Sx = Sy;
		Sy = d;
	}
	else
	{
		invert = 0;
	} //end-else  

	// Now compute Sxx & Sxy
	for (int i = start; i < start + count; i++)
	{
		Sxx += x[i] * x[i];
		Sxy += x[i] * y[i];
	} //end-for

	double D = S * Sxx - Sx * Sx;
	a = (Sxx * Sy - Sx * Sxy) / D;
	b = (S * Sxy - Sx * Sy) / D;

	if (b == 0.0)
	{
		// Vertical or horizontal line
		double error = 0.0;
		for (int i = start; i < start + count; i++)
		{
			error += abs(a - y[i]);
		} //end-for
		e = error / count;
	}
	else
	{
		double error = 0.0;
		for (int i = start; i < start + count; i++)
		{
			// Let the line passing through (x[i], y[i]) that is perpendicular to a+bx be c+dx
			double d = -1.0 / (b);
			double c = y[i] - d * x[i];
			double x2 = (a - c) / (d - b);
			double y2 = a + b * x2;

			double dist = (x[i] - x2) * (x[i] - x2) + (y[i] - y2) * (y[i] - y2);
			error += dist;
		} //end-for

		e = sqrt(error / count);
	} //end-else
} // end LineFit