#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv\cvaux.h>

class ImageProcessing
{
public:
	static void RGBtoHSV(const IplImage* rgbFrame, IplImage* hsvFrame);
	static void erode(const IplImage* sourceFrame, IplImage* erodedFrame);
	static void dilate(const IplImage* sourceFrame, IplImage* dilatedFrame);
	static void smooth(const IplImage* sourceFrame, IplImage* smoothedFrame);
};

#endif // IMAGEPROCESSING_H