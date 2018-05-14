#include "win32_dictymaze.h"

#include "dictymaze.h"
#include "dictymaze.cpp"

#include <windows.h>

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
