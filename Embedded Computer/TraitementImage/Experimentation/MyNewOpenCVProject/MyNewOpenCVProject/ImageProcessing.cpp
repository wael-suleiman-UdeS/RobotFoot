#include "ImageProcessing.h"

using namespace cv;

void ImageProcessing::RGBtoHSV(IplImage* rgbFrame, IplImage* hsvFrame)
{
	if (!rgbFrame || !hsvFrame) {return;}
	cvCvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);
}

void ImageProcessing::erode(IplImage* sourceFrame, IplImage* erodedFrame)
{
	if (!sourceFrame || !erodedFrame) {return;}
	cvErode(sourceFrame, erodedFrame, NULL, 0);
}

void ImageProcessing::dilate(IplImage* sourceFrame, IplImage* dilatedFrame)
{
	if (!sourceFrame || !dilatedFrame) {return;}
	cvDilate(sourceFrame, dilatedFrame, NULL, 2);
}

void ImageProcessing::smooth(IplImage* sourceFrame, IplImage* smoothedFrame)
{
	if (!sourceFrame || !smoothedFrame) {return;}
	cvSmooth(sourceFrame, smoothedFrame, CV_GAUSSIAN, 9, 9);
}	