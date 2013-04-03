#pragma once

#include <opencv\cvaux.h>

class ImageProcessing
{
public:
	static void RGBtoHSV(IplImage* rgbFrame, IplImage* hsvFrame);
	static void erode(IplImage* sourceFrame, IplImage* erodedFrame);
	static void dilate(IplImage* sourceFrame, IplImage* dilatedFrame);
	static void smooth(IplImage* sourceFrame, IplImage* smoothedFrame);
};

