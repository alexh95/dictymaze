#if !defined(DICTYMAZE_OPENCV_H)

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

typedef cv::Mat image;

#define SQRT_2_DIV_2 0.70710678118654752440084436210485

struct point_i32
{
	i32 I;
	i32 J;
};

struct image_object
{
	u32 Label;
	u32 PixelSize;
};

struct histogram
{
	u32 Data[256];
};

#define DICTYMAZE_OPENCV_H
#endif
