#include "dictymaze_opencv.h"

inline point_i32
PointI32(i32 X, i32 Y)
{
	point_i32 Result = {X, Y};
	return Result;
}

inline point_i32
operator+(point_i32 A, point_i32 B)
{
	point_i32 Result;

	Result.X = A.X + B.X;
	Result.Y = A.Y + B.Y;

	return Result;
}

inline point_i32
operator+=(point_i32& A, point_i32 B)
{
	A = A + B;
	return A;
}

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
ImageU8C3(u32 Rows, u32 Cols, pixel_rgb Value = {})
{
	return image(Rows, Cols, CV_8UC3, cv::Scalar(Value.E[2], Value.E[1], Value.E[0]));
}

inline image
ImageU8C3(image* Image, pixel_rgb Value = {})
{
	return ImageU8C3(Image->rows, Image->cols, Value);
}

inline pixel_rgb
GetAtU8C3(image* Image, point_i32 Point)
{
	cv::Vec3b Value = Image->at<cv::Vec3b>(Point.I, Point.J);
	pixel_rgb Result = {Value[2], Value[1], Value[0]};
	return Result;
}

inline void
SetAtU8C3(image* Image, point_i32 Point, pixel_rgb Value)
{
	cv::Vec3b Pixel = {Value.B, Value.G, Value.R};
	Image->at<cv::Vec3b>(Point.I, Point.J) = Pixel;
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
ImageF32(u32 Rows, u32 Cols, f32 Value = 0.f)
{
	return image(Rows, Cols, CV_32FC1, cv::Scalar(Value));
}

inline image
ImageF32(image* Image, f32 Value = 0.f)
{
	return ImageF32(Image->rows, Image->cols, Value);
}

inline f32
GetAtF32(image* Image, point_i32 Point)
{
	return Image->at<f32>(Point.I, Point.J);
}

inline void
SetAtF32(image* Image, point_i32 Point, f32 Value)
{
	Image->at<f32>(Point.I, Point.J) = Value;
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
CopyImage(image* Src, image* Dst)
{
	Src->copyTo(*Dst);
}

inline image
CloneImage(image* Image)
{
	return Image->clone();
}

inline void
CreateWindow(char* WindowName)
{
	cv::namedWindow(WindowName, cv::WINDOW_AUTOSIZE);
}

inline void
DestroyWindow(char* WindowName)
{
	cv::destroyWindow(WindowName);
}

inline image
ReadGrayscaleImage(char* ImageName)
{
	return cv::imread(ImageName, cv::IMREAD_GRAYSCALE);
}

inline void
WriteImage(char* ImageName, image* Image)
{
	Assert(Image->data);
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
Threshold(image* Src, image* Dst, u32 ThresholdValue)
{
	cv::threshold(*Src, *Dst, ThresholdValue, 255, cv::THRESH_TOZERO);
}

inline void
AdaptiveThreshold(image* Src, image* Dst)
{
	Assert(Dst->data);
	cv::adaptiveThreshold(*Src, *Dst, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, 0);
}

inline u32
ConnectedComponents(image* Src, image* Dst)
{
	Assert(Dst->data);
	u32 Result = cv::connectedComponents(*Src, *Dst, 8, CV_32SC1);
	return Result;
}

inline void
ExtractLargestLabeledFeatures(image* Src, image* Dst, image* Labels, u32 LabelCount, u32 MinObjectSize, u32 MaxObjectCount)
{
	Assert(Src->data && Dst->data);
	// TODO(alex): prevent stack overflow
	image_object* Objects = (image_object*)MemoryAllocate((LabelCount + 1) * SizeOf(image_object));
	for (u32 Label = 0; Label <= LabelCount; ++Label)
	{
		Objects[Label].Label = Label;
		Objects[Label].Size = 0;
	}
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = PointI32(Col, Row);
			i32 Label = GetAtI32(Labels, Point);
			if (Label > 0)
			{
				++Objects[Label].Size;
			}
		}
	}

	u32 LargeObjectCount = 0;
	for (u32 ObjectIndex = 0; ObjectIndex <= LabelCount; ++ObjectIndex)
	{
		if (Objects[ObjectIndex].Size >= MinObjectSize)
		{
			++LargeObjectCount;
		}
	}
	image_object* LargeObjects = (image_object*)MemoryAllocate(LargeObjectCount * SizeOf(image_object));
	u32 LargeObjectIndex = 0;
	for (u32 ObjectIndex = 0; ObjectIndex <= LabelCount; ++ObjectIndex)
	{
		if (Objects[ObjectIndex].Size >= MinObjectSize)
		{
			LargeObjects[LargeObjectIndex++] = Objects[ObjectIndex];
		}
	}
	MemoryFree(Objects);

	for (u32 ObjectIndex1 = 0; ObjectIndex1 < LargeObjectCount; ++ObjectIndex1)
	{
		for (u32 ObjectIndex2 = ObjectIndex1 + 1; ObjectIndex2 < LargeObjectCount; ++ObjectIndex2)
		{
			if (LargeObjects[ObjectIndex1].Size < LargeObjects[ObjectIndex2].Size)
			{
				image_object Temp = LargeObjects[ObjectIndex1];
				LargeObjects[ObjectIndex1] = LargeObjects[ObjectIndex2];
				LargeObjects[ObjectIndex2] = Temp;
			}
		}
	}

	u8* LabelValue = (u8*)MemoryAllocate(LabelCount + 1);
	MemoryZero(LabelValue, LabelCount + 1);
	for (u32 ObjectIndex = 0; ObjectIndex < MaxObjectCount; ++ObjectIndex)
	{
		LabelValue[LargeObjects[ObjectIndex].Label] = 255;
	}
	MemoryFree(LargeObjects);

	u32 ObjectsDeleted = LabelCount - MaxObjectCount;
	if (ObjectsDeleted > 0)
	{
		for (i32 Row = 0; Row < Src->rows; ++Row)
		{
			for (i32 Col = 0; Col < Src->cols; ++Col)
			{
				point_i32 Point = PointI32(Col, Row);
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
			point_i32 Point = { Col, Row };
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

		f64* RowData = (f64*)MemoryAllocate(LevelCols * SizeOf(f64));
		f64* TransformRowData = (f64*)MemoryAllocate(LevelCols * SizeOf(f64));
		for (i32 Row = 0; Row < LevelRows; ++Row)
		{
			for (i32 Col = 0; Col < LevelCols; ++Col)
			{
				RowData[Col] = GetAtF64(&Temp, { Col, Row });
			}
			WaveletHaarTransform1D(RowData, TransformRowData, LevelCols);
			for (i32 Col = 0; Col < LevelCols; ++Col)
			{
				SetAtF64(&Temp, { Col, Row }, TransformRowData[Col]);
			}
		}
		MemoryFree(TransformRowData);
		MemoryFree(RowData);

		f64* ColData = (f64*)MemoryAllocate(LevelRows * SizeOf(f64));
		f64* TransformColData = (f64*)MemoryAllocate(LevelRows * SizeOf(f64));
		for (i32 Col = 0; Col < LevelCols; ++Col)
		{
			for (i32 Row = 0; Row < LevelRows; ++Row)
			{
				ColData[Row] = GetAtF64(&Temp, { Col, Row });
			}
			WaveletHaarTransform1D(ColData, TransformColData, LevelRows);
			for (i32 Row = 0; Row < LevelRows; ++Row)
			{
				SetAtF64(&Temp, { Col, Row }, TransformColData[Row]);
			}
		}
		MemoryFree(ColData);
		MemoryFree(TransformColData);
	}

	for (i32 Row = 0; Row < Temp.rows; ++Row)
	{
		for (i32 Col = 0; Col < Temp.cols; ++Col)
		{
			point_i32 Point = { Col, Row };
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
			u8 Value = GetAtU8(Src, {Col, Row});
			++Result.Data[Value];
		}
	}

	return Result;
}

void
ThresholdTop(image* Src, image* Dst, f64 Ratio, b32 IgnoreZero = false)
{
	histogram Histogram = CalculateHistogram(Src);
	u32 ImageSize = Src->rows * Src->cols;
	if (IgnoreZero)
	{
		ImageSize -= Histogram.Data[0];
	}
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
			point_i32 Point = {Col, Row};
			u8 Value = GetAtU8(Src, Point);
			u8 NewValue = (Value >= LastValue) ? Value : 0;
			SetAtU8(Dst, Point, NewValue);
		}
	}
}

void
FillConvexPoly(image* Src, point_i32* Points, u32 PointCount, u8 Color)
{
	cv::fillConvexPoly(*Src, (cv::Point*)Points, PointCount, cv::Scalar(Color));
}

void
AbsoluteDifference(image* Src1, image* Src2, image* Out)
{
	cv::absdiff(*Src1, *Src2, *Out);
}

void
MaxImageF64(image* Src1, image* Src2, image* Dst)
{
	Assert(Src1->data && Src2->data && Dst->data);
	Assert(Src1->rows == Src2->rows && Src1->cols == Src2->cols);
	Assert(Src1->rows == Dst->rows && Src1->cols == Dst->cols);
	for (i32 Row = 0; Row < Src1->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src1->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			f64 Value1 = GetAtF64(Src1, Point);
			f64 Value2 = GetAtF64(Src2, Point);
			f64 Result = MAX(Value1, Value2);
			SetAtF64(Dst, Point, Result);
		}
	}
}

void
GetGaborKernel(image* Dst, point_i32 KernelSize, f64 Sigma, f64 Theta, f64 Lambda, f64 Gamma, f64 Psi, i32 KernelType = CV_64F)
{
	Assert(Dst->data);
	image Kernel = cv::getGaborKernel({KernelSize.I, KernelSize.J}, Sigma, Theta, Lambda, Gamma, Psi, KernelType);
	CopyImage(&Kernel, Dst);
}

void
ApplyConvolutionU8F64(image* Src, image* Dst, image* Kernel)
{
	Assert(Src->data && Dst->data && Kernel->data);
	Assert(Src->rows == Dst->rows && Src->cols == Dst->cols);

	i32 KernelRadiusV = Kernel->rows >> 1;
	i32 KernelRadiusH = Kernel->cols >> 1;
	point_i32 KernelRadius = {KernelRadiusH, KernelRadiusV};

	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			f64 Result = 0.;

			for (i32 KRow = -KernelRadiusV; KRow <= KernelRadiusV; ++KRow)
			{
				for (i32 KCol = -KernelRadiusH; KCol <= KernelRadiusH; ++KCol)
				{
					point_i32 DPoint = {KCol, KRow};
					point_i32 SrcPoint = Point + DPoint;
					point_i32 KPoint = KernelRadius + DPoint;
					if (SrcPoint.I >= 0 && SrcPoint.I < Src->rows && SrcPoint.J >= 0 && SrcPoint.J < Src->cols)
					{
						u8 Value = GetAtU8(Src, SrcPoint);
						f64 KValue = GetAtF64(Kernel, KPoint);
						Result += (f64)Value * KValue;
					}
				}
			}

			SetAtF64(Dst, Point, Result);
		}
	}
}

inline f64
MapMinMax(f64 Value, f64 FromMin, f64 FromMax, f64 ToMin, f64 ToMax)
{
	f64 Result = ToMin + ((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin);
	return Result;
}

void
NormalizeMatF64(image* Src, image* Dst)
{
	Assert(Src->data && Dst->data);
	Assert(Src->rows == Dst->rows && Src->cols == Src->cols);

	f64 Min = 0.;
	f64 Max = 0.;
	cv::minMaxLoc(*Src, &Min, &Max);
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			f64 Value = GetAtF64(Src, Point);
			f64 MappedValue = MapMinMax(Value, Min, Max, 0., 1.);
			SetAtF64(Dst, Point, MappedValue);
		}
	}
}

f64
MeanF64(image* Src, b32 IncludeZero = true, f64 Epsilon = 0.0001)
{
	Assert(Src->data);
	f64 Result = 0.;
	f64 Sum = 0.;
	i32 Count = 0;
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			f64 Value = GetAtF64(Src, Point);
			if (IncludeZero || Abs(Value) >= Epsilon)
			{
				Sum += Value;
				++Count;
			}
		}
	}
	if (Count > 0)
	{
		Result = Sum / (f64)Count;
	}
	return Result;
}

void
F64ToU8(image* Src, image* Dst)
{
	Assert(Src->data && Dst->data);
	Assert(Src->rows == Dst->rows && Src->cols == Dst->cols);

	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			f64 Value = GetAtF64(Src, Point);
			u8 NewValue = (u8)MapMinMax(Value, 0., 1., 0., 255.);
			SetAtU8(Dst, Point, NewValue);
		}
	}
}

void
DifferenceCellFilter(image* Src, image* Dst)
{
	Assert(Src->data && Dst->data);
	Assert(Src->rows == Dst->rows && Src->cols == Dst->cols);

	image Filtered = ImageF64(Dst);

	image Kernel1 = ImageF64(31, 31);
	image Kernel2 = ImageF64(31, 31);
	GetGaborKernel(&Kernel1, {31, 31}, 8., 0., 1., 4., 0.);
	GetGaborKernel(&Kernel2, {31, 31}, 8., PI * 0.5, 1., 4., 0.);

	image Kernel = ImageF64(31, 31);
	MaxImageF64(&Kernel1, &Kernel2, &Kernel);
	// ApplyConvolutionU8F64(Src, Dst, &Kernel);
	cv::filter2D(*Src, Filtered, CV_64F, Kernel, {-1, -1}, 0., cv::BORDER_CONSTANT);
	NormalizeMatF64(&Filtered, &Filtered);
	F64ToU8(&Filtered, Dst);
}

void
HistogramDraw(image* Dst, histogram* Histogram, u32 From = 0, u32 To = 256)
{
	Assert(Dst->data);
	Assert(To - From > 0);

	u32 Min = Histogram->Data[From];
	u32 Max = Histogram->Data[From];
	for (u32 Index = From + 1; Index < To; ++Index)
	{
		u32 Value = Histogram->Data[Index];
		if (Value < Min)
		{
			Min = Value;
		}
		if (Value > Max)
		{
			Max = Value;
		}
	}

	u32 BarWidth = Dst->cols / ArrayCount(Histogram->Data);
	u32 BarWidthOffset = 0;
	if (BarWidth > 1)
	{
		BarWidthOffset = BarWidth / 2;
	}

	cv::rectangle(*Dst, {0, 0}, {Dst->cols, Dst->rows}, cv::Scalar(0), CV_FILLED);

	for (u32 Index = From; Index < To; ++Index)
	{
		u32 Value = Histogram->Data[Index];
		u32 BarHeight = (u32)MapMinMax(Value, Min, Max, 0, Dst->rows);

		cv::Point Point1 = {(i32)(Index * BarWidth), (i32)(Dst->rows - BarHeight)};
		cv::Point Point2 = {(i32)((Index + 1) * BarWidth - BarWidthOffset), (i32)(Dst->rows)};
		cv::rectangle(*Dst, Point1, Point2, cv::Scalar(255), CV_FILLED);
	}
}

void
ExtractCandidateCells(image* CellsImage, image* CandidateCellLabels, candidate_cell* CandidateCells, u32 CandidateCellCount)
{
	for (i32 LabelIndex = 0; LabelIndex < CandidateCellCount; ++LabelIndex)
	{
		candidate_cell* CandidateCell = CandidateCells + LabelIndex;
		CandidateCell->Label = LabelIndex + 1;
		CandidateCell->BoundingBox.TopLeft = {CellsImage->cols, CellsImage->rows};
		CandidateCell->BoundingBox.BottomRight = {};
		CandidateCell->Center = {};
		CandidateCell->Size = 0;
		CandidateCell->WeightedSize = 0.;
		CandidateCell->Scores = 0;
		CandidateCell->ScoreCount = 0;
	}

	for (i32 Row = 0; Row < CellsImage->rows; ++Row)
	{
		for (i32 Col = 0; Col < CellsImage->cols; ++Col)
		{
			point_i32 Point = {Col, Row};
			i32 LabelIndex = GetAtI32(CandidateCellLabels, Point) - 1;
			if (LabelIndex >= 0)
			{
				u8 Value = GetAtU8(CellsImage, Point);

				if (Row < CandidateCells[LabelIndex].BoundingBox.TopLeft.I)
				{
					CandidateCells[LabelIndex].BoundingBox.TopLeft.I = Row;
				}
				if (Col < CandidateCells[LabelIndex].BoundingBox.TopLeft.J)
				{
					CandidateCells[LabelIndex].BoundingBox.TopLeft.J = Col;
				}

				if (Row > CandidateCells[LabelIndex].BoundingBox.BottomRight.I)
				{
					CandidateCells[LabelIndex].BoundingBox.BottomRight.I = Row;
				}
				if (Col > CandidateCells[LabelIndex].BoundingBox.BottomRight.J)
				{
					CandidateCells[LabelIndex].BoundingBox.BottomRight.J = Col;
				}

				CandidateCells[LabelIndex].Center.X += (f32)Col;
				CandidateCells[LabelIndex].Center.Y += (f32)Row;
				CandidateCells[LabelIndex].Size += 1;
				CandidateCells[LabelIndex].WeightedSize += (f32)Value / 255.;
			}
		}
	}

	for (i32 LabelIndex = 0; LabelIndex < CandidateCellCount; ++LabelIndex)
	{
		u32 Size = CandidateCells[LabelIndex].Size;
		CandidateCells[LabelIndex].Center.X /= (f32)Size;
		CandidateCells[LabelIndex].Center.Y /= (f32)Size;
	}
}

void
CellTrackerInit(cell_tracker* CellTracker, candidate_cell* CandidateCell, u32 StateIndex, u32 StateCount)
{
	CellTracker->KalmanFilter.init(4, 2, 0);

	cv::setIdentity(CellTracker->KalmanFilter.transitionMatrix);
	CellTracker->KalmanFilter.transitionMatrix.at<f32>(0, 2) = 1.f;
	CellTracker->KalmanFilter.transitionMatrix.at<f32>(1, 3) = 1.f;

	CellTracker->KalmanFilter.statePre.setTo(0.f);
	CellTracker->KalmanFilter.statePre.at<f32>(0) = CandidateCell->Center.X;
	CellTracker->KalmanFilter.statePre.at<f32>(1) = CandidateCell->Center.Y;

	CellTracker->KalmanFilter.statePost.setTo(0.f);
	CellTracker->KalmanFilter.statePost.at<f32>(0) = CandidateCell->Center.X;
	CellTracker->KalmanFilter.statePost.at<f32>(1) = CandidateCell->Center.Y;

	cv::setIdentity(CellTracker->KalmanFilter.measurementMatrix);
	cv::setIdentity(CellTracker->KalmanFilter.processNoiseCov, cv::Scalar::all(0.001f));
	cv::setIdentity(CellTracker->KalmanFilter.measurementNoiseCov, cv::Scalar::all(0.1f));
	cv::setIdentity(CellTracker->KalmanFilter.errorCovPost, cv::Scalar::all(0.1f));

	u32 PredictedPositionsSize = StateCount * SizeOf(v2);
	CellTracker->PredictedPositions = (v2*)MemoryAllocate(PredictedPositionsSize);
	MemoryZero(CellTracker->PredictedPositions, PredictedPositionsSize);

	u32 EstimatedPositionsSize = StateCount * SizeOf(v2);
	CellTracker->EstimatedPositions = (v2*)MemoryAllocate(EstimatedPositionsSize);
	MemoryZero(CellTracker->EstimatedPositions, EstimatedPositionsSize);

	u32 ActualPositionsSize = StateCount * SizeOf(v2);
	CellTracker->ActualPositions = (v2*)MemoryAllocate(ActualPositionsSize);
	MemoryZero(CellTracker->ActualPositions, ActualPositionsSize);

	u32 BoundingBoxesSize = StateCount * SizeOf(rect_i32);
	CellTracker->BoundingBoxes = (rect_i32*)MemoryAllocate(BoundingBoxesSize);
	MemoryZero(CellTracker->BoundingBoxes, BoundingBoxesSize);

	u32 SizesSize = StateCount * SizeOf(f32);
	CellTracker->Sizes = (f32*)MemoryAllocate(SizesSize);
	MemoryZero(CellTracker->Sizes, SizesSize);

	CellTracker->ActualPositions[StateIndex] = CandidateCell->Center;
	CellTracker->Sizes[StateIndex] = CandidateCell->WeightedSize;

	CellTracker->StateCount = StateCount;
	CellTracker->BoundingBoxes[StateIndex] = CandidateCell->BoundingBox;
}

void
CellTrackerDestroy(cell_tracker* CellTracker)
{
	MemoryFree(CellTracker->PredictedPositions);
	MemoryFree(CellTracker->EstimatedPositions);
	MemoryFree(CellTracker->ActualPositions);
	MemoryFree(CellTracker->BoundingBoxes);
	MemoryFree(CellTracker->Sizes);
}

v2
CellTrackerPredict(cell_tracker* CellTracker, u32 StateIndex)
{
	Assert(StateIndex > 0 && StateIndex < CellTracker->StateCount);

	image Prediction = CellTracker->KalmanFilter.predict();
	v2 PredictedPoint = {Prediction.at<f32>(0), Prediction.at<f32>(1)};
	CellTracker->PredictedPositions[StateIndex] = PredictedPoint;
	return PredictedPoint;
}

v2
CellTrackerCorrect(cell_tracker* CellTracker, u32 StateIndex, candidate_cell* CandidateCell)
{
	Assert(StateIndex > 0 && StateIndex < CellTracker->StateCount);

	image Correction = ImageF32(2, 1);
	Correction.at<f32>(0) = (CandidateCell) ? CandidateCell->Center.X : CellTracker->ActualPositions[StateIndex - 1].X;
	Correction.at<f32>(1) = (CandidateCell) ? CandidateCell->Center.Y : CellTracker->ActualPositions[StateIndex - 1].Y;
	
	image Estimate = CellTracker->KalmanFilter.correct(Correction);
	v2 EstimatedPoint = {Estimate.at<f32>(0), Estimate.at<f32>(1)};
	CellTracker->EstimatedPositions[StateIndex] = EstimatedPoint;

	CellTracker->ActualPositions[StateIndex] = (CandidateCell) ? CandidateCell->Center : EstimatedPoint;
	CellTracker->Sizes[StateIndex] = (CandidateCell) ? CandidateCell->WeightedSize : CellTracker->Sizes[StateIndex - 1];
	if (CandidateCell)
	{
		CellTracker->BoundingBoxes[StateIndex] = CandidateCell->BoundingBox;
	}

	return EstimatedPoint;
}

inline f32
CellTrackerLastSize(cell_tracker* CellTracker, u32 StateIndex)
{
	Assert(StateIndex > 0 && StateIndex < CellTracker->StateCount);
	f32 Result = CellTracker->Sizes[StateIndex - 1];
	Assert(Result > 0.f);
	return Result;
}
