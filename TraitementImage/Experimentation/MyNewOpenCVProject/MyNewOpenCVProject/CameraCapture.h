#pragma once

#include <opencv\highgui.h>
#include "ImageProcessing.h"

class CameraCapture
{
private:
	// todo : fichier ini
	static const int DEVICE_INDEX  = 1;
	static const int WIDTH  = 640;
	static const int HEIGHT = 480;

	static CameraCapture* _instance;

	IplImage* _rgbFrame;
	IplImage* _hsvFrame;
	CvCapture* _capture;

	CameraCapture();
	~CameraCapture();

	void ProcessFrame();

public:
	enum class ColorSpace {RGB, HSV};

	static CameraCapture* GetInstance();
	void CaptureFrame();

	IplImage* GetFrame(ColorSpace colorSpace);

};