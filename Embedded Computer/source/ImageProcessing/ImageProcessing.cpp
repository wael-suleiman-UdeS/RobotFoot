#include "ImageProcessing.h"

using namespace cv;

void ImageProcessing::RGBtoHSV(const IplImage* rgbFrame, IplImage* hsvFrame)
{
	if (!rgbFrame || !hsvFrame) { return; }
	cvCvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);
}

void ImageProcessing::erode(const IplImage* sourceFrame, IplImage* erodedFrame, int iterations)
{
	if (!sourceFrame || !erodedFrame) { return; }
	cvErode(sourceFrame, erodedFrame, nullptr, iterations);
}

void ImageProcessing::dilate(const IplImage* sourceFrame, IplImage* dilatedFrame, int iterations)
{
	if (!sourceFrame || !dilatedFrame) { return; }
	cvDilate(sourceFrame, dilatedFrame, nullptr, iterations);
}

void ImageProcessing::smooth(const IplImage* sourceFrame, IplImage* smoothedFrame, int apertureSize)
{
	if (!sourceFrame || !smoothedFrame) { return; }
	cvSmooth(sourceFrame, smoothedFrame, CV_GAUSSIAN, apertureSize);
}	
