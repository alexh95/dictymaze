#if !defined(DICTYMAZE_OPENCV_H)

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "dictymaze_math.h"

typedef cv::Mat image;

typedef cv::KalmanFilter kalman_filter;

#define SQRT_2_DIV_2 0.70710678118654752440084436210485

struct pixel_rgb
{
	union
	{
		struct
		{
			u8 R;
			u8 G;
			u8 B;
		};
		u8 E[3];
	};
};

pixel_rgb HighlightColors[] =
{
	{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255},
	{255, 128, 0}, {255, 0, 128}, {0, 255, 128}, {128, 255, 0}, {0, 128, 255}, {128, 0, 255}
};

struct point_i32
{
	union
	{
		struct
		{
			i32 J;
			i32 I;
		};
		struct
		{
			i32 X;
			i32 Y;
		};
	};
};

struct rect_i32
{
	point_i32 TopLeft;
	point_i32 BottomRight;
};

struct image_object
{
	u32 Label;
	u32 Size;
};

struct histogram
{
	u32 Data[256];
};

struct candidate_cell
{
	u32 Label;
	rect_i32 BoundingBox;
	v2 Center;
	u32 Size;
	f32 WeightedSize;
};

struct analyzed_candidate_cell
{
	candidate_cell* CandidateCell;
	f32 Score;
};

struct cell_tracker
{
	kalman_filter KalmanFilter;
	v2* PredictedPositions;
	v2* EstimatedPositions;
	v2* ActualPositions;
	rect_i32* BoundingBoxes;
	f32* Sizes;
	u32 StateCount;
};

#define DICTYMAZE_OPENCV_H
#endif
