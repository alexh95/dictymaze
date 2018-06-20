#include "dictymaze.h"
#include "dictymaze_math.cpp"
#include "dictymaze_opencv.cpp"

void
StringCopy(char* Dst, char* Src, u32 Count)
{
	for (u32 Index = 0; Index < Count; ++Index)
	{
		Dst[Index] = Src[Index];
	}
}

u32
StringLength(char* String)
{
	u32 Result = 0;

	if (String)
	{
		while (String[Result++]);
	}

	return Result;
}

image_set
GetImageSet(char* Name, char* Directory, b32 Load = false)
{
	image_set Result = {};

	StringCopy(Result.Name, Name, StringLength(Name));
	StringCopy(Result.Directory, Directory, StringLength(Directory));
	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		if (Name && Load)
		{
			char FileName[256] = {};
			sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", Result.Directory, Result.Name, Result.Name, ImageIndex);
			Result.Images[ImageIndex] = ReadGrayscaleImage(FileName);
		}
		else
		{
			Result.Images[ImageIndex] = image();
		}
	}

	return Result;
}

image*
GetImage(image_set* ImageSet, u32 ImageIndex)
{
	image* Result = ImageSet->Images + ImageIndex;

	if (!Result->data)
	{
		if (StringLength(ImageSet->Name))
		{
			char FileName[256] = {};
			sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
			ImageSet->Images[ImageIndex] = ReadGrayscaleImage(FileName);
			Result = ImageSet->Images + ImageIndex;
		}
	}

	return Result;
}

void
SaveImageSet(image_set* ImageSet)
{
	char FileName[256] = {};
	sprintf(FileName, "data");
	MakeDirectory(FileName);

	sprintf(FileName, "data\\%s", ImageSet->Directory);
	MakeDirectory(FileName);
	
	sprintf(FileName, "data\\%s\\%s", ImageSet->Directory, ImageSet->Name);
	MakeDirectory(FileName);

	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
		WriteImage(FileName, GetImage(ImageSet, ImageIndex));
	}
}

void
StabilizeImages(image_set* DstImageSet, image_set* SrcImageSet)
{
	char OutputWindowName[] = "Image Stabilization";
	CreateWindow(OutputWindowName);
	image StabilizationDisplay = ImageU8(480, 640);
	cv::putText(StabilizationDisplay, "Stabilizing images", {220, 240}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255));
	ShowImage(OutputWindowName, &StabilizationDisplay);
	WaitKey(1);

	image* PrevImage = GetImage(SrcImageSet, 0);
	image PrevImageEq = ImageU8(PrevImage);
	EqualizeHistogram(PrevImage, &PrevImageEq);
	v3 Trajectory = {};
	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		image* NextImage = GetImage(SrcImageSet, ImageIndex);
		image NextImageEq = ImageU8(NextImage);
		EqualizeHistogram(NextImage, &NextImageEq);

		v2 PrevCorners[MAX_CORNERS] = {};
		v2 NextCorners[MAX_CORNERS] = {};
		u32 FilteredCornerCount = 0;
		GoodFeaturesToTrack(&NextImageEq, PrevCorners, MAX_CORNERS, 0.005, 10);
		CalculateOpticalFlowPiramidsLucasKanade(&PrevImageEq, &NextImageEq, PrevCorners, NextCorners, MAX_CORNERS, &FilteredCornerCount);
		v3 Transform = EstimateRigidTransform(PrevCorners, NextCorners, FilteredCornerCount, false);
		Trajectory += Transform;
		image* DstImage = GetImage(DstImageSet, ImageIndex);
		WarpAffine(DstImage, NextImage, -Trajectory);

		PrevImageEq = NextImageEq;
	}

	cv::rectangle(StabilizationDisplay, {0, 0}, {640, 480}, cv::Scalar(0), CV_FILLED);
	cv::putText(StabilizationDisplay, "Saving images", {220, 240}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255));
	ShowImage(OutputWindowName, &StabilizationDisplay);
	WaitKey(1);

	SaveImageSet(DstImageSet);
	DestroyWindow(OutputWindowName);
}

b32
GetStabilizedImages(image_set* ImageSet, char* ImageSetName)
{
	*ImageSet = GetImageSet(ImageSetName, "Stabilized");
	// for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		image* StabilizedImage = GetImage(ImageSet, 0 /*ImageIndex*/);
		if (StabilizedImage->data == 0)
		{
			return false;
		}
	}
	return true;
}

inline u32
PrevImageIndex(u32 ImageIndex)
{
	i32 PrevImageIndex = ImageIndex - 1;
	if (PrevImageIndex < 0)
	{
		PrevImageIndex = IMAGE_SET_SIZE - 1;
	}
	return PrevImageIndex;
}

inline u32
NextImageIndex(u32 ImageIndex)
{
	u32 NextImageIndex = ImageIndex + 1;
	if (NextImageIndex >= IMAGE_SET_SIZE)
	{
		NextImageIndex = 0;
	}
	return NextImageIndex;
}

inline b32
PointInBounds(image* Image, point_i32 Point)
{
	b32 Result = (Point.I >= 0 && Point.I < Image->rows) && (Point.J >= 0 && Point.J < Image->cols);
	return Result;
}

image
ExtractMaze(image* Image)
{
	image Maze = CloneImage(Image);
	EqualizeHistogram(&Maze, &Maze);
	Laplacian(&Maze, &Maze);
	MorphOpen(&Maze, &Maze, 3);
	AdaptiveThreshold(&Maze, &Maze);
	MorphClose(&Maze, &Maze, 3);
	MorphOpen(&Maze, &Maze, 3);
	image MazeLabels = ImageI32(&Maze);
	u32 LabelCount = ConnectedComponents(&Maze, &MazeLabels);
	ExtractLargestLabeledFeatures(&Maze, &Maze, &MazeLabels, LabelCount, 15, 30);
	MorphClose(&Maze, &Maze, 5);
	// TODO(alex): fill the small holes
	return Maze;
}

void
Dictymaze()
{
	char ImageSetName[] = "ax2__52517_2";
	image_set ImageSet = GetImageSet(ImageSetName, "SourceSplit");

	image_set StabilizedImageSet;
	if (!GetStabilizedImages(&StabilizedImageSet, ImageSetName))
	{
		StabilizeImages(&StabilizedImageSet, &ImageSet);
	}

	char OutputWindowName1[] = "Output 1";
	CreateWindow(OutputWindowName1);

	char OutputWindowName2[] = "Output 2";
	CreateWindow(OutputWindowName2);

	b32 Running = true;
	b32 Paused = true;
	u32 FrameTime = 33;
	u32 ImageIndex = 101;
	while (Running)
	{
		// image* Image = GetImage(&ImageSet, ImageIndex);
		image* StabilizedPrevImage = GetImage(&StabilizedImageSet, PrevImageIndex(ImageIndex));
		image StabilizedPrevImageEq = CloneImage(StabilizedPrevImage);
		EqualizeHistogram(&StabilizedPrevImageEq, &StabilizedPrevImageEq);
		image* StabilizedImage = GetImage(&StabilizedImageSet, ImageIndex);
		image StabilizedImageEq = CloneImage(StabilizedImage);
		EqualizeHistogram(&StabilizedImageEq, &StabilizedImageEq);

		image Maze = ExtractMaze(&StabilizedImageEq);
		image AreaOfInterestMask = ImageU8(StabilizedImage);
		point_i32 AreaOfInterest[4] = {{82, 22}, {72, 335}, {930, 343}, {940, 47}};
		FillConvexPoly(&AreaOfInterestMask, AreaOfInterest, ArrayCount(AreaOfInterest), 255);
		AreaOfInterestMask = AreaOfInterestMask & Maze;

		image Difference = ImageU8(StabilizedImage);
		AbsoluteDifference(&StabilizedImageEq, &StabilizedPrevImageEq, &Difference);
		Difference = Difference & AreaOfInterestMask;

		ShowImage(OutputWindowName1, &Difference);

		ThresholdTop(&Difference, &Difference, 0.0075);
		ShowImage(OutputWindowName2, &Difference);

		image Cells = ImageF64(&Difference);
		DifferenceCellFilter(&Difference, &Cells);

		u32 KeyCode = WaitKey(Paused ? 0 : FrameTime);
		switch (KeyCode)
		{
			case KEY_SPACE:
			{
				Paused = !Paused;
			} break;
			case KEY_ESCAPE:
			{
				Running = false;				
			} break;
			case KEY_R:
			{
				ImageIndex = 1;
			} break;
			case KEY_S:
			{
				ImageIndex = 1;
				Paused = true;
			} break;
			case KEY_RIGHT:
			{
				ImageIndex = NextImageIndex(ImageIndex);
			} break;
			case KEY_LEFT:
			{
				ImageIndex = PrevImageIndex(ImageIndex);
			} break;
			case -1:
			{
				ImageIndex = NextImageIndex(ImageIndex);
			} break;
			default: 
			{
				int a = KeyCode;
			} break;
		}
	}
}
