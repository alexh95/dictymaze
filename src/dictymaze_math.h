#if !defined(DICTYMAZE_MATH_H)

#define PI 3.14159265358979

#define Abs(Value) ((Value) > 0 ? Value : -Value)

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

#define DICTYMAZE_MATH_H
#endif
