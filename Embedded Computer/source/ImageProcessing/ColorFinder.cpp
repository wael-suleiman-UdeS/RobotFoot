#include "ColorFinder.h"
#include <climits>

ColorFinder::ColorFinder(const HSVcolor* color)
{
	_color = color;
}

CvPoint ColorFinder::getCirclePosition(const Mat& frame, CircleSpec spec)
{
	CvPoint circlePosition = {-1, -1};

	if (frame.empty()) { return circlePosition; }

	filter(frame);

	ImageProcessing::erode(_resultFrame, _resultFrame, spec.erosionIterations);
	ImageProcessing::dilate(_resultFrame, _resultFrame, spec.dilationIterations);
	ImageProcessing::smooth(_resultFrame, _resultFrame, spec.smoothingApertureSize);
	
	vector<Vec3f> circles;
	HoughCircles(_resultFrame, circles, CV_HOUGH_GRADIENT,
		spec.resolutionDivisor, spec.minDistance, spec.edgeThreshold,
		spec.centerThreshold, spec.minRadius, spec.maxRadius);

	if (circles.size())
	{
		circlePosition.x = circles[0][0];
		circlePosition.y = circles[0][1];
	}

	return circlePosition;
}

void ColorFinder::setColor(const HSVcolor* color)
{
	_color = color;
}

void ColorFinder::filter(const Mat& sourceFrame)
{
	if (sourceFrame.empty() || !_color) { return; }

	Scalar minHSV = cvScalar(_color->hue - _color->hueTolerance, _color->saturation, _color->brightness);
	Scalar maxHSV = cvScalar(_color->hue + _color->hueTolerance, UCHAR_MAX, UCHAR_MAX);

	ImageProcessing::filter(sourceFrame, _resultFrame, minHSV, maxHSV);
}
