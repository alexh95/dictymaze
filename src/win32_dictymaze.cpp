#include "win32_dictymaze.h"

#include "dictymaze.h"
#include "dictymaze.cpp"

#include <windows.h>

void*
AllocateOnStack(size Size)
{
    return _alloca(Size);
}

void*
AllocateOnStackSafe(size Size)
{
	void* Result = _malloca(Size);
    return Result;
}

void
FreeOnStackSafe(void* Buffer)
{
	_freea(Buffer);
}

void
MemoryZero(void* Buffer, size Length)
{
	ZeroMemory(Buffer, Length);
}

void
MakeDirectory(char* PathName)
{
	CreateDirectory(PathName, 0);
}

void*
AllocateMemory(size Size)
{
    void* Result = malloc(Size);
    return Result;
}

void
FreeMemory(void* Buffer)
{
    free(Buffer);
}

int CALLBACK WinMain(
    HINSTANCE hInstance,
    HINSTANCE hPrevInstance,
    LPSTR lpCmdLine,
    int nCmdShow
)
{
    Dictymaze();
    
    return 0;
}
