#if !defined(DICTYMAZE_PLATFORM_H)

#include <stdint.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef i32 b32;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef float f32;
typedef double f64;

#define ArrayCount(Array) (sizeof(Array)/(sizeof(Array[0])))

#define Min(A, B) ((A < B) ? (A) : (B))

void
Dictymaze();

#define DICTYMAZE_PLATFORM_H
#endif
