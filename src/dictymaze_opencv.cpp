#include "dictymaze_opencv.h"
#include "dictymaze_math.h"

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
ShowImage(char* WindowName, image* Image)
{
	cv::imshow(WindowName, *Image);
}

inline i32 
WaitKey(u32 Timeout)
{
	return cv::waitKeyEx(Timeout);
}

inline void
EqualizeHistogram(image* InputImage, image* OutputImage)
{
	cv::equalizeHist(*InputImage, *OutputImage);
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

inline void
EstimateRigidTransform(
	m3* Transform,
	v2* Src,
	v2* Dst,
	u32 Count,
	b32 FullAffine)
{
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
		Transform->D[0][0] = TransformInternal.at<f64>(0, 0);
		Transform->D[0][1] = TransformInternal.at<f64>(0, 1);
		Transform->D[0][2] = TransformInternal.at<f64>(0, 2);
		Transform->D[1][0] = TransformInternal.at<f64>(1, 0);
		Transform->D[1][1] = TransformInternal.at<f64>(1, 1);
		Transform->D[1][2] = TransformInternal.at<f64>(1, 2);
	}
}

inline void
CopyImage(image* Dst, image* Src)
{
	Src->copyTo(*Dst);
}
