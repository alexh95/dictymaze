#if !defined(DICTYMAZE_H)

#include "dictymaze_platform.h"
#include "dictymaze_opencv.h"

#define IMAGE_SET_SIZE 1000

#define KEY_SPACE 32
#define KEY_ESCAPE 27
#define KEY_RIGHT 2555904
#define KEY_UP 2490368
#define KEY_LEFT 2424832
#define KEY_DOWN 2621440

struct image_set
{
	char Name[256];
	image Images[IMAGE_SET_SIZE];
};

#define DICTYMAZE_H
#endif
