#if !defined(DICTYMAZE_OPENCV_H)

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

typedef cv::Mat image;

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

struct point_f32
{
	union
	{
		struct
		{
			f32 J;
			f32 I;
		};
		struct
		{
			f32 X;
			f32 Y;
		};
	};
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
	point_i32 TopLeft;
	point_i32 BottomRight;
	point_i32 Center;
	u32 Size;
	f32 WeightedSize;
};

#define DICTYMAZE_OPENCV_H
#endif
