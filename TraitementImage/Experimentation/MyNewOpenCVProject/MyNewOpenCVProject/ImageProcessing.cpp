#include "ImageProcessing.h"

using namespace cv;

void ImageProcessing::RGBtoHSV(IplImage* rgbFrame, IplImage* hsvFrame)
{
	cvCvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);
}

void ImageProcessing::Erode(IplImage* sourceFrame, IplImage* erodedFrame)
{
	cvErode(sourceFrame, erodedFrame, NULL, 0);
}

void ImageProcessing::Dilate(IplImage* sourceFrame, IplImage* dilatedFrame)
{
	cvDilate(sourceFrame, dilatedFrame, NULL, 2);
}

void ImageProcessing::Smooth(IplImage* sourceFrame, IplImage* smoothedFrame)
{
	cvSmooth(sourceFrame, smoothedFrame, CV_GAUSSIAN, 9, 9);
}	