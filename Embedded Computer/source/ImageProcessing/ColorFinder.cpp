#include "ColorFinder.h"
#include <climits>

using namespace cv;

ColorFinder::ColorFinder(const HSVcolor* color)
{
	_color = color;
}

CvPoint ColorFinder::getCirclePosition(const IplImage* frame, CircleSpec spec)
{
	CvPoint circlePosition = {-1, -1};

	if (!frame) { return circlePosition; }

	filter(frame);

	ImageProcessing::erode(_resultFrame, _resultFrame, spec.erosionIterations);
	ImageProcessing::dilate(_resultFrame, _resultFrame, spec.dilationIterations);
	ImageProcessing::smooth(_resultFrame, _resultFrame, spec.smoothingApertureSize);

	CvMemStorage* storage = cvCreateMemStorage(0);
	
	CvSeq* circles = cvHoughCircles(_resultFrame, storage, CV_HOUGH_GRADIENT,
		spec.resolutionDivisor, spec.minDistance, spec.edgeThreshold,
		spec.centerThreshold, spec.minRadius, spec.maxRadius);

	if (circles->total)
	{
		float* positionBuffer = (float*)cvGetSeqElem(circles, 0);
		circlePosition.x = positionBuffer[0];
		circlePosition.y = positionBuffer[1];
	}

	cvReleaseMemStorage(&storage);

	return circlePosition;
}

void ColorFinder::setColor(const HSVcolor* color)
{
	_color = color;
}

void ColorFinder::filter(const IplImage* sourceFrame)
{
	if (!sourceFrame || !_color) { return; }

	CvScalar minHSV = cvScalar(_color->hue - _color->hueTolerance, _color->saturation, _color->brightness, 0);
	CvScalar maxHSV = cvScalar(_color->hue + _color->hueTolerance, UCHAR_MAX, UCHAR_MAX, 0);

	CvSize frameSize = cvSize(sourceFrame->width, sourceFrame->height);
	_resultFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 1);

	cvInRangeS(sourceFrame, minHSV, maxHSV, _resultFrame);
}
