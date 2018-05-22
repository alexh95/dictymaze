#include "dictymaze_opencv.h"
#include "dictymaze_math.h"

inline image
Image(u32 Rows, u32 Cols, u32 Format)
{
	return image(Rows, Cols, Format);
}

inline void
CopyImage(image* Dst, image* Src)
{
	Src->copyTo(*Dst);
}

inline image
CloneImage(image* Image)
{
	return Image->clone();
}

inline void
CreateNamedWindow(char* WindowName)
{
	cv::namedWindow(WindowName, cv::WINDOW_AUTOSIZE);
}

inline image
ReadGrayscaleImage(char* ImageName)
{
	return cv::imread(ImageName, cv::IMREAD_GRAYSCALE);
}

inline void
WriteImage(char* ImageName, image* Image)
{
	cv::imwrite(ImageName, *Image);
}

inline void 
ShowImage(char* WindowName, image* Image)
{
	cv::imshow(WindowName, *Image);
}

inline i32 
WaitKey(u32 Timeout)
{
	return cv::waitKeyEx(Timeout);
}

inline image
EqualizeHistogram(image* SrcImage)
{
	image Result;

	cv::equalizeHist(*SrcImage, Result);

	return Result;
}

inline void
GoodFeaturesToTrack(
	image* Image, 
	v2* Corners, 
	u32 MaxCorners, 
	f64 QualityLevel, 
	f64 MinDistance)
{
	cv::Mat CornersInternal;

	cv::goodFeaturesToTrack(*Image, CornersInternal, MaxCorners, QualityLevel, MinDistance);

	for (u32 CornerIndex = 0; CornerIndex < CornersInternal.rows; ++CornerIndex)
	{
		Corners[CornerIndex] = v2{CornersInternal.at<f32>(CornerIndex, 0), CornersInternal.at<f32>(CornerIndex, 1)};
	}
}

inline void
CalculateOpticalFlowPiramidsLucasKanade(
	image* PrevImage,
	image* NextImage,
	v2* PrevCorners,
	v2* NextCorners,
	u32 CornerCount,
	u32* FilteredCornerCount)
{
	cv::Mat PrevCornersInternal(CornerCount, 2, CV_32FC1);
	cv::Mat NextCornersInternal;
	cv::Mat Status;
	cv::Mat Error;

	for (u32 CornerIndex = 0; CornerIndex < CornerCount; ++CornerIndex)
	{
		PrevCornersInternal.at<f32>(CornerIndex, 0) = PrevCorners[CornerIndex].X;
		PrevCornersInternal.at<f32>(CornerIndex, 1) = PrevCorners[CornerIndex].Y;
	}

	cv::calcOpticalFlowPyrLK(*PrevImage, *NextImage, PrevCornersInternal, NextCornersInternal, Status, Error);

	u32 FilteredCount = 0;
	for (u32 StatusIndex = 0; StatusIndex < Status.rows; ++StatusIndex)
	{
		u8 StatusValue = Status.at<u8>(StatusIndex);
		if (StatusValue)
		{
			PrevCorners[FilteredCount] = v2{PrevCornersInternal.at<f32>(StatusIndex, 0), PrevCornersInternal.at<f32>(StatusIndex, 1)};
			NextCorners[FilteredCount] = v2{NextCornersInternal.at<f32>(StatusIndex, 0), NextCornersInternal.at<f32>(StatusIndex, 1)};
			++FilteredCount;
		}
	}
	*FilteredCornerCount = FilteredCount;
}

inline v3
EstimateRigidTransform(
	v2* Src,
	v2* Dst,
	u32 Count,
	b32 FullAffine)
{
	v3 Result = {};

	cv::Mat SrcInternal(Count, 2, CV_32FC1);
	cv::Mat DstInternal(Count, 2, CV_32FC1);
	for (u32 Index = 0; Index < Count; ++Index)
	{
		SrcInternal.at<f32>(Index, 0) = Src[Index].X;
		SrcInternal.at<f32>(Index, 1) = Src[Index].Y;
		DstInternal.at<f32>(Index, 0) = Dst[Index].X;
		DstInternal.at<f32>(Index, 1) = Dst[Index].Y;
	}

	cv::Mat TransformInternal = cv::estimateRigidTransform(SrcInternal, DstInternal, FullAffine);

	if (TransformInternal.data)
	{
		Result.X = TransformInternal.at<f64>(0, 2);
		Result.Y = TransformInternal.at<f64>(1, 2);
		Result.Z = atan2(TransformInternal.at<f64>(1, 0), TransformInternal.at<f64>(0, 0));
	}

	return Result;
}

inline void WarpAffine(image* Dst, image* Src, v3 Transform)
{
	cv::Mat Matrix(2, 3, CV_32FC1);
	Matrix.at<f32>(0, 0) = cos(Transform.Z);
	Matrix.at<f32>(1, 0) = -sin(Transform.Z);
	Matrix.at<f32>(0, 1) = sin(Transform.Z);
	Matrix.at<f32>(1, 1) = cos(Transform.Z);
	Matrix.at<f32>(0, 2) = Transform.X;
	Matrix.at<f32>(1, 2) = Transform.Y;
	cv::warpAffine(*Src, *Dst, Matrix, Src->size());
}
