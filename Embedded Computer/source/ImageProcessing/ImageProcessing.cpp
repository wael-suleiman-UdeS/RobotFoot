#include "ImageProcessing.h"

/** \brief Convert a BGR frame to an HSV frame
 *
 * \param bgrFrame const Mat&: BGR frame to convert
 * \param hsvFrame Mat&: Destination HSV frame
 *
 */
void ImageProcessing::BGRtoHSV(const Mat& bgrFrame, Mat& hsvFrame)
{
	cvtColor(bgrFrame, hsvFrame, CV_BGR2HSV);
}

/** \brief Filter a frame by bounds
 *
 * \param sourceFrame const Mat&: Frame to filter
 * \param filteredFrame Mat&: Destination filtered frame
 * \param lowerBound Scalar: Lower bound of the filter ([H,S,V] if in the HVS space)
 * \param upperBound Scalar: Upper bound of the filter ([H,S,V] if in the HVS space)
 *
 */
void ImageProcessing::filter(const Mat& sourceFrame, Mat& filteredFrame,
							 Scalar lowerBound, Scalar upperBound)
{
	inRange(sourceFrame, lowerBound, upperBound, filteredFrame);
}

/** \brief Apply erosion on a frame a specified number of times
 *
 * \param sourceFrame const Mat&: Frame to erode
 * \param erodedFrame Mat&: Destination eroded frame
 * \param iterations int: Number of times to apply erosion
 *
 */
void ImageProcessing::erode(const Mat& sourceFrame, Mat& erodedFrame, int iterations)
{
	cv::erode(sourceFrame, erodedFrame, iterations);
}

/** \brief Apply dilation on a frame a specified number of times
 *
 * \param sourceFrame const Mat&: Frame to dilate
 * \param dilatedFrame Mat&: Destination dilated frame
 * \param iterations int: Number of times to apply dilation
 *
 */
void ImageProcessing::dilate(const Mat& sourceFrame, Mat& dilatedFrame, int iterations)
{
	cv::dilate(sourceFrame, dilatedFrame, iterations);
}

/** \brief Smooth a frame using Gaussian Blur
 *
 * \param sourceFrame const Mat&: Frame to smooth
 * \param smoothedFrame Mat&: Destination smoothed frame
 * \param apertureSize int: The Gaussian kernel size (must be positive and odd)
 *
 */
void ImageProcessing::smooth(const Mat& sourceFrame, Mat& smoothedFrame, int apertureSize)
{
	GaussianBlur(sourceFrame, smoothedFrame, Size(apertureSize, apertureSize), 0, 0);
}