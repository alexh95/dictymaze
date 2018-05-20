#include "dictymaze_math.h"

v2
operator+(v2 A, v2 B)
{
	v2 Result;

	Result.X = A.X + B.X;
	Result.Y = A.Y + B.Y;

	return Result;
}

v2
operator+=(v2& A, v2 B)
{
	A = A + B;
	return A;
}

v3
operator-(v3 A)
{
	v3 Result;

	Result.X = -A.X;
	Result.Y = -A.Y;
	Result.Z = -A.Z;

	return Result;
}

v3
operator+(v3 A, v3 B)
{
	v3 Result;

	Result.X = A.X + B.X;
	Result.Y = A.Y + B.Y;
	Result.Z = A.Z + B.Z;

	return Result;
}

v3
operator+=(v3& A, v3 B)
{
	A = A + B;
	return A;
}

m3
operator+(m3 A, m3 B)
{
	m3 Result;

	Result.D[0][0] = A.D[0][0] + B.D[0][0];
	Result.D[0][1] = A.D[0][1] + B.D[0][1];
	Result.D[0][2] = A.D[0][2] + B.D[0][2];
	Result.D[1][0] = A.D[1][0] + B.D[1][0];
	Result.D[1][1] = A.D[1][1] + B.D[1][1];
	Result.D[1][2] = A.D[1][2] + B.D[1][2];
	Result.D[2][0] = A.D[2][0] + B.D[2][0];
	Result.D[2][1] = A.D[2][1] + B.D[2][1];
	Result.D[2][2] = A.D[2][2] + B.D[2][2];

	return Result;
}

m3
operator+=(m3& A, m3 B)
{
	A = A + B;
	return A;
}
