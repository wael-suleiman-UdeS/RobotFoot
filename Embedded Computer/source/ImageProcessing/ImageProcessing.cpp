#include "ImageProcessing.h"

void ImageProcessing::RGBtoHSV(const Mat& rgbFrame, Mat& hsvFrame)
{
	cvtColor(rgbFrame, hsvFrame, CV_RGB2HSV);
}

void ImageProcessing::filter(const Mat& sourceFrame, Mat& filteredFrame,
							 Scalar lowerThreshold, Scalar upperThreshold)
{
	inRange(sourceFrame, lowerThreshold, upperThreshold, filteredFrame);
}

void ImageProcessing::erode(const Mat& sourceFrame, Mat& erodedFrame, int iterations)
{
	cv::erode(sourceFrame, erodedFrame, iterations);
}

void ImageProcessing::dilate(const Mat& sourceFrame, Mat& dilatedFrame, int iterations)
{
	cv::dilate(sourceFrame, dilatedFrame, iterations);
}

void ImageProcessing::smooth(const Mat& sourceFrame, Mat& smoothedFrame, int apertureSize)
{
	GaussianBlur(sourceFrame, smoothedFrame, Size(apertureSize, apertureSize), 0, 0);
}
