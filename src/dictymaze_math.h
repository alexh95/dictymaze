#if !defined(DICTYMAZE_MATH_H)

struct v2
{
	union
	{
		struct
		{
			f32 X, Y;
		};
		struct
		{
			f32 E[2];
		};
	};
};

struct m3
{
	union
	{
		struct
		{
			f32 D[3][3];
		};
		struct
		{
			f32 E[9];
		};
	};
};

v2
V2(f32 X, f32 Y)
{
	v2 Result = {};
	Result.X = X;
	Result.Y = Y;
	return Result;
}

#define DICTYMAZE_MATH_H
#endif
