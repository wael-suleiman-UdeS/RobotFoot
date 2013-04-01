#pragma once

#include <opencv\cvaux.h>

class ImageProcessing
{
public:
	static void RGBtoHSV(IplImage* rgbFrame, IplImage* hsvFrame);
	static void Erode(IplImage* sourceFrame, IplImage* erodedFrame);
	static void Dilate(IplImage* sourceFrame, IplImage* dilatedFrame);
	static void Smooth(IplImage* sourceFrame, IplImage* smoothedFrame);
};

