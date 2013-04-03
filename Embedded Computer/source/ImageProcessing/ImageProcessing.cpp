#include "ImageProcessing.h"

using namespace cv;

void ImageProcessing::RGBtoHSV(const IplImage* rgbFrame, IplImage* hsvFrame)
{
	if (!rgbFrame || !hsvFrame) { return; }
	cvCvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);
}

void ImageProcessing::erode(const IplImage* sourceFrame, IplImage* erodedFrame)
{
	if (!sourceFrame || !erodedFrame) { return; }
	cvErode(sourceFrame, erodedFrame, nullptr, 0);
}

void ImageProcessing::dilate(const IplImage* sourceFrame, IplImage* dilatedFrame)
{
	if (!sourceFrame || !dilatedFrame) { return; }
	cvDilate(sourceFrame, dilatedFrame, nullptr, 2);
}

void ImageProcessing::smooth(const IplImage* sourceFrame, IplImage* smoothedFrame)
{
	if (!sourceFrame || !smoothedFrame) { return; }
	cvSmooth(sourceFrame, smoothedFrame, CV_GAUSSIAN, 9, 9);
}	
