#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv\cvaux.h>

class ImageProcessing
{
public:
	static void RGBtoHSV(const IplImage* rgbFrame, IplImage* hsvFrame);
	static void erode(const IplImage* sourceFrame, IplImage* erodedFrame, int iterations);
	static void dilate(const IplImage* sourceFrame, IplImage* dilatedFrame, int iterations);
	static void smooth(const IplImage* sourceFrame, IplImage* smoothedFrame, int apertureSize);
};

#endif // IMAGEPROCESSING_H