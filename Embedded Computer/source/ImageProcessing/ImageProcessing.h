#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv\cvaux.h>

using namespace cv;

class ImageProcessing
{
public:
	static void RGBtoHSV(const Mat& rgbFrame, Mat& hsvFrame);
	static void filter(const Mat& sourceFrame, Mat& filteredFrame,
		Scalar lowerThreshold, Scalar upperThreshold);
	static void erode(const Mat& sourceFrame, Mat& erodedFrame, int iterations);
	static void dilate(const Mat& sourceFrame, Mat& dilatedFrame, int iterations);
	static void smooth(const Mat& sourceFrame, Mat& smoothedFrame, int apertureSize);
};

#endif // IMAGEPROCESSING_H