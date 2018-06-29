#include "win32_dictymaze.h"

#include "dictymaze.h"
#include "dictymaze.cpp"

#include <windows.h>

void*
MemoryAllocate(size Size)
{
    void* Result = malloc(Size);
    return Result;
}

void
MemoryFree(void* Buffer)
{
    free(Buffer);
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
