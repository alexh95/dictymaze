#if !defined(DICTYMAZE_MATH_H)

#define PI 3.14159265358979

#define Abs(Value) ((Value) > 0 ? Value : -Value)

#define Min(A, B) (((A) < (B)) ? (A) : (B))

#define Max(A, B) (((A) < (B)) ? (A) : (B))

#define Sqr(A) ((A) * (A))

#define Sort(Array, Type, Size, Comparator)\
for (u32 Index1 = 0; Index1 < Size - 1; ++Index1)\
{\
	for (u32 Index2 = Index1 + 1; Index2 < Size; ++Index2)\
	{\
		Type* A = Array + Index1;\
		Type* B = Array + Index2;\
		if (Comparator(A, B))\
		{\
			Type Temp = *A;\
			*A = *B;\
			*B = Temp;\
		}\
	}\
}

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
