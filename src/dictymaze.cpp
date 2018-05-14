#include "dictymaze.h"
#include "dictymaze_math.cpp"
#include "dictymaze_opencv.cpp"

#define MAX_CORNERS 256

image
GetImage(image_set* ImageSet, u32 ImageIndex)
{
	image result = ImageSet->Images[ImageIndex];

	if (!result.data)
	{
		char FileName[256] = {};
		sprintf(FileName, "../data/%s/%s_%d.tif", ImageSet->Name, ImageSet->Name, ImageIndex + 1);
		result = ImageSet->Images[ImageIndex] = ReadGrayscaleImage(FileName);
	}

	return result;
}

void
StringCopy(char* Dst, char* Src, u32 Count)
{
	for (u32 Index = 0; Index < Count; ++Index)
	{
		Dst[Index] = Src[Index];
	}
}

void
StabilizeImages(image_set* ImageSet)
{
	m3 Transforms[ArrayCount(ImageSet->Images)];
	m3 PrevTransform = {};
	image PrevImage = GetImage(ImageSet, 0);
	EqualizeHistogram(&PrevImage, &PrevImage);
	for (u32 ImageIndex = 1; ImageIndex < ArrayCount(ImageSet->Images); ++ImageIndex)
	{
		image NextImage = GetImage(ImageSet, ImageIndex);
		EqualizeHistogram(&NextImage, &NextImage);

		v2 PrevCorners[MAX_CORNERS] = {};
		v2 NextCorners[MAX_CORNERS] = {};
		u32 FilteredCornerCount = 0;

		GoodFeaturesToTrack(&NextImage, PrevCorners, MAX_CORNERS, 0.01, 32);
		CalculateOpticalFlowPiramidsLucasKanade(&PrevImage, &NextImage, PrevCorners, NextCorners, MAX_CORNERS, &FilteredCornerCount);

		m3 Transform = {};
		EstimateRigidTransform(&Transform, PrevCorners, NextCorners, FilteredCornerCount, false);
		if (Transform.D[0][0] != 0.f)
		{
			PrevTransform = Transform;			
		}
		else
		{
			Transform = PrevTransform;
		}

		Transforms[ImageIndex] = Transform;
		CopyImage(&PrevImage, &NextImage);
	}
}

void
Dictymaze()
{
	image_set ImageSet = {};
	char ImageSetName[] = "ax2__52517_2";
	StringCopy(ImageSet.Name, ImageSetName, ArrayCount(ImageSetName));
	
	StabilizeImages(&ImageSet);

	char OutputWindowName[] = "Output";
	CreateNamedWindow(OutputWindowName);
	b32 Paused = false;
	u32 FrameTime = 33;
	b32 Running = true;
	u32 ImageIndex = 0;
	while (Running)
	{
		image Image = GetImage(&ImageSet, ImageIndex);

		u32 NextImageIndex = ImageIndex + 1;
		if (NextImageIndex >= IMAGE_SET_SIZE)
		{
			NextImageIndex = 0;
		}

		ShowImage(OutputWindowName, &Image);
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
			case KEY_RIGHT:
			{
				ImageIndex = NextImageIndex;
			} break;
			case KEY_LEFT:
			{
				i32 PrevImageIndex = ImageIndex - 1;
				if (PrevImageIndex < 0)
				{
					PrevImageIndex = IMAGE_SET_SIZE - 1;
				}

				ImageIndex = PrevImageIndex;

			} break;
			case -1:
			{
				ImageIndex = NextImageIndex;
			} break;
			default: 
			{
				int a = KeyCode;
			} break;
		}
	}
}
