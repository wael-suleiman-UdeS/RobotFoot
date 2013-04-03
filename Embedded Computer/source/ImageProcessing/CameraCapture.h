#pragma once

#include <opencv\highgui.h>
#include "ImageProcessing.h"

class CameraCapture
{
public:
	enum class ColorSpace {RGB, HSV};

	static CameraCapture& getInstance();
	void captureFrame();

	IplImage* getFrame(ColorSpace colorSpace);

private:
	// todo : fichier ini
	static const int DEVICE_INDEX  = 1;
	static const int WIDTH  = 640;
	static const int HEIGHT = 480;

	IplImage* _rgbFrame;
	IplImage* _hsvFrame;
	CvCapture* _capture;

	CameraCapture();
	CameraCapture(const CameraCapture&);
	~CameraCapture();

	void processFrame();
};