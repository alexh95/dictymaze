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
			sprintf(FileName, "../data/%s/%s/%s_%d.tif", Result.Directory, Result.Name, Result.Name, ImageIndex);
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
			sprintf(FileName, "../data/%s/%s/%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
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
	sprintf(FileName, "../data");
	MakeDirectory(FileName);

	sprintf(FileName, "../data/%s", ImageSet->Directory);
	MakeDirectory(FileName);
	
	sprintf(FileName, "../data/%s/%s", ImageSet->Directory, ImageSet->Name);
	MakeDirectory(FileName);

	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		sprintf(FileName, "../data/%s/%s/%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
		WriteImage(FileName, GetImage(ImageSet, ImageIndex));
	}
}

void
StabilizeImages(image_set* DstImageSet, image_set* SrcImageSet)
{
	image* PrevImage = GetImage(SrcImageSet, 0);
	image PrevImageEq = EqualizeHistogram(PrevImage);
	v3 Trajectory = {};
	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		image* NextImage = GetImage(SrcImageSet, ImageIndex);
		image NextImageEq = EqualizeHistogram(NextImage);

		v2 PrevCorners[MAX_CORNERS] = {};
		v2 NextCorners[MAX_CORNERS] = {};
		u32 FilteredCornerCount = 0;
		GoodFeaturesToTrack(&NextImageEq, PrevCorners, MAX_CORNERS, 0.01, 32);
		CalculateOpticalFlowPiramidsLucasKanade(&PrevImageEq, &NextImageEq, PrevCorners, NextCorners, MAX_CORNERS, &FilteredCornerCount);
		v3 Transform = EstimateRigidTransform(PrevCorners, NextCorners, FilteredCornerCount, false);
		Trajectory += Transform;
		image* DstImage = GetImage(DstImageSet, ImageIndex);
		WarpAffine(DstImage, NextImage, -Trajectory);

		PrevImageEq = NextImageEq;
	}

	SaveImageSet(DstImageSet);
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

image
ImageDifference(image* A, image* B)
{
	return *A - *B;
}

image
Threshold(image* Image, u32 Threshold)
{
	return *Image > Threshold;
}

image
Laplacian(image* Image)
{
	image Result = CloneImage(Image);

	cv::Laplacian(*Image, Result, CV_8U, 3);

	return Result;
}

image
Invert(image* Image)
{
	image Result = CloneImage(Image);

	cv::bitwise_not(*Image, Result);

	return Result;
}

image
AdaptiveThreshold(image* Image)
{
	image Result;

	cv::adaptiveThreshold(*Image, Result, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5, 0);

	return Result;
}

image
MorphOpen(image* Image, u32 KernelSize)
{
	image Result;

	cv::morphologyEx(*Image, Result, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize, KernelSize)));

	return Result;
}

image
MorphClose(image* Image, u32 KernelSize)
{
	image Result;

	cv::morphologyEx(*Image, Result, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize, KernelSize)));

	return Result;
}

inline b32
PointInBounds(image* Image, point_i32 Point)
{
	b32 Result = (Point.I >= 0 && Point.I < Image->rows) && (Point.J >= 0 && Point.J < Image->cols);
	return Result;
}

u32
Label(image* Src, image* Labels)
{
	i32 LabelCount = 0;
	point_i32 Neighbours[4] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}};
	for (i32 Row = 0; Row < Src->rows; ++Row)
	{
		for (i32 Col = 0; Col < Src->cols; ++Col)
		{
			point_i32 Point = PointI32(Row, Col);
			u8 Value = GetAtU8(Src, Point);

			if (Value > 0)
			{
				b32 LabelNotSet = true;
				for (i32 NeighbourIndex = 0; NeighbourIndex < 4; ++NeighbourIndex)
				{
					point_i32 NeighbourPoint = Point + Neighbours[NeighbourIndex];
					if (PointInBounds(Labels, NeighbourPoint))
					{
						i32 NeighbourLabel = GetAtI32(Labels, NeighbourPoint);
						if (NeighbourLabel > 0)
						{
							SetAtI32(Labels, Point, NeighbourLabel);
							LabelNotSet = false;
							break;
						}
					}
				}
				if (LabelNotSet) 
				{
					SetAtI32(Labels, Point, ++LabelCount);
				}
			}
		}
	}

	return LabelCount;
}

void
ExtractLargestLabeledFeatures(image* Src, image* Labels, u32 LabelCount, image* Dst)
{
	// TODO(alex): implement largest object filtering
}

image
ExtractMaze(image* Image)
{
	image Maze = EqualizeHistogram(Image);
	Maze = Laplacian(&Maze);
	Maze = MorphOpen(&Maze, 3);
	Maze = AdaptiveThreshold(&Maze);
	Maze = MorphClose(&Maze, 3);
	Maze = MorphOpen(&Maze, 3);
	image MazeLabels = ImageI32(&Maze);
	u32 LabelCount = Label(&Maze, &MazeLabels);
	ExtractLargestLabeledFeatures(&Maze, &MazeLabels, LabelCount, &Maze);
	// ShowImage("Output 1", &Maze);
	// ShowImage("Output 2", &Maze);
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
	CreateNamedWindow(OutputWindowName1);

	char OutputWindowName2[] = "Output 2";
	CreateNamedWindow(OutputWindowName2);

	b32 Running = true;
	b32 Paused = true;
	u32 FrameTime = 33;
	u32 ImageIndex = 1;
	while (Running)
	{
		// image* Image = GetImage(&ImageSet, ImageIndex);
		image* PrevStabilizedImage = GetImage(&StabilizedImageSet, PrevImageIndex(ImageIndex));
		image* StabilizedImage = GetImage(&StabilizedImageSet, ImageIndex);

		image Difference = ImageDifference(StabilizedImage, PrevStabilizedImage);
		ShowImage(OutputWindowName1, StabilizedImage);

		image Maze = ExtractMaze(StabilizedImage);
		ShowImage(OutputWindowName2, &Maze);

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
