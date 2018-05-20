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

struct v3
{
	union
	{
		struct
		{
			f32 X, Y, Z;
		};
		struct
		{
			f32 E[3];
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
	v2 Result = v2{X, Y};
	return Result;
}

v3
V3(f32 X, f32 Y, f32 Z)
{
	v3 Result = v3{X, Y, Z};
	return Result;
}

#define DICTYMAZE_MATH_H
#endif
