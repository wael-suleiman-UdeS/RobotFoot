#pragma once

#include <opencv\highgui.h>
#include "ImageProcessing.h"

class Camera
{
public:
	enum class ColorSpace {RGB, HSV};

	static Camera& getInstance();
	bool initialize();
	void captureFrame();

	IplImage* getFrame(ColorSpace colorSpace);

private:
	// todo : fichier ini
	static const int DEVICE_INDEX  = 0;
	static const int WIDTH  = 640;
	static const int HEIGHT = 480;

	IplImage* _rgbFrame;
	IplImage* _hsvFrame;
	CvCapture* _capture;

	Camera() {}
	Camera(const Camera&);
	~Camera();

	void processFrame();
};