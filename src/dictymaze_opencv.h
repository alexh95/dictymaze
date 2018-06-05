#if !defined(DICTYMAZE_OPENCV_H)

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

typedef cv::Mat image;

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

#define DICTYMAZE_OPENCV_H
#endif
