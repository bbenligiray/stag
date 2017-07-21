#ifndef CURVATURECORNERDETECTION_H
#define CURVATURECORNERDETECTION_H

#include "ED/EdgeMap.h"
#include <cmath>
#include <vector>

std::vector<int> DetectAndValidateCurvatureCorners(const EdgeSegment &edgeSeg);

inline void SmoothSegment(double *dst, double *src, int len);
inline void DerivateSegment(double *dst, double *src, int len);
inline void LineFit(double *x, double *y, int start, int count, double &a, double &b, double &e, int &invert);

#endif