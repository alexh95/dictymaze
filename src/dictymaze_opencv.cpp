#include "dictymaze_opencv.h"
#include "dictymaze_math.h"

inline image
ImageU8(u32 Rows, u32 Cols, u8 Value = 0)
{
	return image(Rows, Cols, CV_8UC1, cv::Scalar(Value));
}

inline image
ImageU8(image* Image, u8 Value = 0)
{
	return ImageU8(Image->rows, Image->cols, Value);
}

inline u8
GetAtU8(image* Image, point_i32 Point)
{
	u8 Result = Image->at<u8>(Point.I, Point.J);
	return Result;
}

inline void
SetAtU8(image* Image, point_i32 Point, u8 Value)
{
	Image->at<u8>(Point.I, Point.J) = Value;
}

inline image
ImageI32(u32 Rows, u32 Cols, i32 Value = 0)
{
	return image(Rows, Cols, CV_32SC1, cv::Scalar(Value));
}

inline image
ImageI32(image* Image, i32 Value = 0)
{
	return ImageI32(Image->rows, Image->cols, Value);
}

inline i32
GetAtI32(image* Image, point_i32 Point)
{
	i32 Result = Image->at<i32>(Point.I, Point.J);
	return Result;
}

inline void
SetAtI32(image* Image, point_i32 Point, i32 Value)
{
	Image->at<i32>(Point.I, Point.J) = Value;
}

inline image
ImageF64(u32 Rows, u32 Cols, f64 Value = 0.)
{
	return image(Rows, Cols, CV_64FC1, cv::Scalar(Value));
}

inline image
ImageF64(image* Image, f64 Value = 0.)
{
	return ImageF64(Image->rows, Image->cols, Value);
}

inline f64
GetAtF64(image* Image, point_i32 Point)
{
	return Image->at<f64>(Point.I, Point.J);
}

inline void
SetAtF64(image* Image, point_i32 Point, f64 Value)
{
	Image->at<f64>(Point.I, Point.J) = Value;
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

inline void
EqualizeHistogram(image* Src, image* Dst)
{
	Assert(Dst->data);
	cv::equalizeHist(*Src, *Dst);
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

inline void
WarpAffine(image* Dst, image* Src, v3 Transform)
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

inline void
MorphOpen(image* Src, image* Dst, u32 KernelSize)
{
	Assert(Dst->data);
	cv::morphologyEx(*Src, *Dst, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize, KernelSize)));
}

inline void
MorphClose(image* Src, image* Dst, u32 KernelSize)
{
	Assert(Dst->data);
	cv::morphologyEx(*Src, *Dst, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize, KernelSize)));
}

inline void
AdaptiveThreshold(image* Src, image* Dst)
{
	Assert(Dst->data);
	cv::adaptiveThreshold(*Src, *Dst, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5, 0);
}

inline u32
ConnectedComponents(image* Src, image* Dst)
{
	Assert(Dst->data);
	u32 Result = cv::connectedComponents(*Src, *Dst, 8, CV_32SC1);
	return Result;
}

inline point_i32
PointI32(i32 I, i32 J)
{
	point_i32 Result = point_i32{I, J};
	return Result;
}

inline point_i32
operator+(point_i32 A, point_i32 B)
{
	point_i32 Result;

	Result.I = A.I + B.I;
	Result.J = A.J + B.J;

	return Result;
}

inline point_i32
operator+=(point_i32& A, point_i32 B)
{
	A = A + B;
	return A;
}

inline void
ExtractLargestLabeledFeatures(image* Src, image* Dst, image* Labels, u32 LabelCount, u32 MinObjectSize, u32 MaxObjectCount)
{
	Assert(Dst->data);
	// TODO(alex): prevent stack overflow
	image_object* Objects = (image_object*)AllocateOnStack((LabelCount + 1) * SizeOf(image_object));
	for (u32 Label = 0; Label <= LabelCount; ++Label)
	{
		Objects[Label].Label = Label;
		Objects[Label].PixelSize = 0;
	}
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = PointI32(Row, Col);
			i32 Label = GetAtI32(Labels, Point);
			if (Label > 0)
			{
				++Objects[Label].PixelSize;
			}
		}
	}

	u32 LargeObjectCount = 0;
	for (u32 ObjectIndex = 0; ObjectIndex <= LabelCount; ++ObjectIndex)
	{
		if (Objects[ObjectIndex].PixelSize >= MinObjectSize)
		{
			++LargeObjectCount;
		}
	}
	image_object* LargeObjects = (image_object*)AllocateOnStack(LargeObjectCount * SizeOf(image_object));
	u32 LargeObjectIndex = 0;
	for (u32 ObjectIndex = 0; ObjectIndex <= LabelCount; ++ObjectIndex)
	{
		if (Objects[ObjectIndex].PixelSize >= MinObjectSize)
		{
			LargeObjects[LargeObjectIndex++] = Objects[ObjectIndex];
		}
	}
	FreeOnStack(Objects);

	for (u32 ObjectIndex1 = 0; ObjectIndex1 < LargeObjectCount; ++ObjectIndex1)
	{
		for (u32 ObjectIndex2 = ObjectIndex1 + 1; ObjectIndex2 < LargeObjectCount; ++ObjectIndex2)
		{
			if (LargeObjects[ObjectIndex1].PixelSize < LargeObjects[ObjectIndex2].PixelSize)
			{
				image_object Temp = LargeObjects[ObjectIndex1];
				LargeObjects[ObjectIndex1] = LargeObjects[ObjectIndex2];
				LargeObjects[ObjectIndex2] = Temp;
			}
		}
	}

	u8* LabelValue = (u8*)AllocateOnStack(LabelCount + 1);
	MemoryZero(LabelValue, LabelCount + 1);
	for (u32 ObjectIndex = 0; ObjectIndex < MaxObjectCount; ++ObjectIndex)
	{
		LabelValue[LargeObjects[ObjectIndex].Label] = 255;
	}
	FreeOnStack(LargeObjects);

	u32 ObjectsDeleted = LabelCount - MaxObjectCount;
	if (ObjectsDeleted > 0)
	{
		for (i32 Row = 0; Row < Src->rows; ++Row)
		{
			for (i32 Col = 0; Col < Src->cols; ++Col)
			{
				point_i32 Point = PointI32(Row, Col);
				i32 Label = GetAtI32(Labels, Point);
				SetAtU8(Dst, Point, LabelValue[Label]);
			}
		}
	}
}

inline void
Laplacian(image* Src, image* Dst)
{
	Assert(Dst->data);
	cv::Laplacian(*Src, *Dst, CV_8U, 3);
}

#if 0
inline void
WaveletHaarTransform1D(f64* Src, f64* Dst, u32 Length)
{
	u32 SrcLength = Length >> 1;
	for (u32 Index = 0; Index < SrcLength; ++Index)
	{
		u32 SrcIndex = Index << 1;
		Dst[Index] = (Src[SrcIndex] + Src[SrcIndex + 1]) * SQRT_2_DIV_2;
		Dst[Index + SrcLength] = (Src[SrcIndex] - Src[SrcIndex + 1]) * SQRT_2_DIV_2;
	}
}

void
WaveletHaarTransform2D(image* Src, image* Dst, u32 Iterations = 1)
{
	Assert(Iterations > 0);

	image Temp = ImageF64(Src);
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = { Row, Col };
			u8 Value = GetAtU8(Src, Point);
			f64 DstValue = (2. * (f64)Value) / 255. - 1.;
			SetAtF64(&Temp, Point, DstValue);
		}
	}

	for (i32 Iteration = 0; Iteration < Iterations; ++Iteration)
	{
		i32 Level = 1 << Iteration;
		i32 LevelRows = Temp.rows / Level;
		i32 LevelCols = Temp.cols / Level;

		f64* RowData = (f64*)AllocateOnStack(LevelCols * SizeOf(f64));
		f64* TransformRowData = (f64*)AllocateOnStack(LevelCols * SizeOf(f64));
		for (i32 Row = 0; Row < LevelRows; ++Row)
		{
			for (i32 Col = 0; Col < LevelCols; ++Col)
			{
				RowData[Col] = GetAtF64(&Temp, { Row, Col });
			}
			WaveletHaarTransform1D(RowData, TransformRowData, LevelCols);
			for (i32 Col = 0; Col < LevelCols; ++Col)
			{
				SetAtF64(&Temp, { Row, Col }, TransformRowData[Col]);
			}
		}
		FreeOnStack(TransformRowData);
		FreeOnStack(RowData);

		f64* ColData = (f64*)AllocateOnStack(LevelRows * SizeOf(f64));
		f64* TransformColData = (f64*)AllocateOnStack(LevelRows * SizeOf(f64));
		for (i32 Col = 0; Col < LevelCols; ++Col)
		{
			for (i32 Row = 0; Row < LevelRows; ++Row)
			{
				ColData[Row] = GetAtF64(&Temp, { Row, Col });
			}
			WaveletHaarTransform1D(ColData, TransformColData, LevelRows);
			for (i32 Row = 0; Row < LevelRows; ++Row)
			{
				SetAtF64(&Temp, { Row, Col }, TransformColData[Row]);
			}
		}
		FreeOnStack(ColData);
		FreeOnStack(TransformColData);
	}

	for (i32 Row = 0; Row < Temp.rows; ++Row)
	{
		for (i32 Col = 0; Col < Temp.cols; ++Col)
		{
			point_i32 Point = { Row, Col };
			f64 Value = GetAtF64(&Temp, Point);
			if (Value > 1.)
			{
				Value = 1.;
			}
			else if (Value < -1.)
			{
				Value = -1.;
			}
			u8 DstValue = (u8)((255. * (Value + 1.)) / 2.);
			SetAtU8(Dst, Point, DstValue);
		}
	}
}
#endif

histogram
CalculateHistogram(image* Src)
{
	histogram Result = {};

	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			u8 Value = GetAtU8(Src, {Row, Col});
			++Result.Data[Value];
		}
	}

	return Result;
}

void
ThresholdTop(image* Src, image* Dst, f64 Ratio)
{
	histogram Histogram = CalculateHistogram(Src);
	u32 ImageSize = Src->rows * Src->cols;
	i32 TargetSize = (i32)((f64)ImageSize * Ratio);
	u32 LastValue = 255;
	while (TargetSize > 0)
	{
		TargetSize -= Histogram.Data[LastValue--];
	}

	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = {Row, Col};
			u8 Value = GetAtU8(Src, Point);
			u8 NewValue = (Value >= LastValue) ? Value : 0;
			SetAtU8(Dst, Point, NewValue);
		}
	}
}
