#include "ColorFinder.h"
#include <climits>

using namespace cv;

ColorFinder::ColorFinder(const HSVcolor* color)
{
	_color = color;
}

CvPoint ColorFinder::getCirclePosition(const IplImage* frame)
{
	CvPoint circlePosition = {-1, -1};

	if (!frame) { return circlePosition; }

	filter(frame);

	ImageProcessing::erode(_resultFrame, _resultFrame);
	ImageProcessing::dilate(_resultFrame, _resultFrame);
	ImageProcessing::smooth(_resultFrame, _resultFrame);

	double resolution = 2;
	double minDistance = _resultFrame->height/4;
	double edgeThreshold = 100;
	double centerThreshold = 10;
	double minRadius = 10;
	double maxRadius = 400;

	CvMemStorage* storage = cvCreateMemStorage(0);
	
	CvSeq* circles = cvHoughCircles(_resultFrame, storage, CV_HOUGH_GRADIENT,
		resolution, minDistance, edgeThreshold, centerThreshold, minRadius, maxRadius);

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
