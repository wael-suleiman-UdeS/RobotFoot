#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv/cvaux.h>
#include <opencv/highgui.h>
/** @addtogroup Image Processing
 * @{
 */

/** \brief Static class for simple image processing operations
 */
class ImageProcessing
{
public:
	static void BGRtoHSV(const cv::Mat& rgbFrame, cv::Mat& hsvFrame);
	static void filter(const cv::Mat& sourceFrame, cv::Mat& filteredFrame,
		cv::Scalar lowerBound, cv::Scalar upperBound);
	static void erode(const cv::Mat& sourceFrame, cv::Mat& erodedFrame, int iterations);
	static void dilate(const cv::Mat& sourceFrame, cv::Mat& dilatedFrame, int iterations);
	static void smooth(const cv::Mat& sourceFrame, cv::Mat& smoothedFrame, int apertureSize);
};

#endif // IMAGEPROCESSING_H
