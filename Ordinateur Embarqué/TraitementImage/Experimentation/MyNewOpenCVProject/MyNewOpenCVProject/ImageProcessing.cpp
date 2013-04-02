#include "ImageProcessing.h"

using namespace cv;

void ImageProcessing::RGBtoHSV(IplImage* rgbFrame, IplImage* hsvFrame)
{
	cvCvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);
}

void ImageProcessing::erode(IplImage* sourceFrame, IplImage* erodedFrame)
{
	cvErode(sourceFrame, erodedFrame, NULL, 0);
}

void ImageProcessing::dilate(IplImage* sourceFrame, IplImage* dilatedFrame)
{
	cvDilate(sourceFrame, dilatedFrame, NULL, 2);
}

void ImageProcessing::smooth(IplImage* sourceFrame, IplImage* smoothedFrame)
{
	cvSmooth(sourceFrame, smoothedFrame, CV_GAUSSIAN, 9, 9);
}	