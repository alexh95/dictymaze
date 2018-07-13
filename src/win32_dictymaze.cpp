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

file
FileOpen(char* FileName)
{
    file Result = {};
    
    OFSTRUCT Ofstruct;
    HFILE Handle = OpenFile(FileName, &Ofstruct, OF_READ);

    if (Handle)
    {
        LARGE_INTEGER FileSize;
        GetFileSizeEx((HANDLE)Handle, &FileSize);

        Result.Handle = (void*)Handle;
        Result.Size = FileSize.QuadPart;
    }

    return Result;

}

u64
FileRead(file* File, void* Buffer, u64 BytesToRead)
{
    DWORD BytesRead = 0;
    ReadFile((HANDLE)File->Handle, Buffer, BytesToRead, &BytesRead, 0);

    u64 Result = BytesRead;
    return Result;
}

int CALLBACK WinMain(
    HINSTANCE hInstance,
    HINSTANCE hPrevInstance,
    LPSTR lpCmdLine,
    int nCmdShow
)
{
    char ParametersFileName[256] = {};
    if (strlen(lpCmdLine) > 0)
    {
        snprintf(ParametersFileName, ArrayCount(ParametersFileName), "%s", (char*)lpCmdLine);
    }
    else
    {
        snprintf(ParametersFileName, ArrayCount(ParametersFileName), "parameters.txt");
    }

    file File = FileOpen(ParametersFileName);
    char FileBytes[256] = {};
    FileRead(&File, FileBytes, File.Size);

    dictymaze_parameters DictymazeParameters = {};
    char* SplitTok = strtok(FileBytes, "\n\r");
    u32 FieldIndex = 0;
    while (SplitTok != 0)
    {
        switch(FieldIndex)
        {
            case 0: {
                snprintf(DictymazeParameters.ImageSetName, ArrayCount(DictymazeParameters.ImageSetName), "%s", SplitTok);
            } break;
            case 1: {
                snprintf(DictymazeParameters.ImageSetLocation, ArrayCount(DictymazeParameters.ImageSetLocation), "%s", SplitTok);
            } break;
            case 2: {
                sscanf(SplitTok, "%d", &DictymazeParameters.ImageLazyLoading);
            } break;
            case 3: {
                sscanf(SplitTok, "%d", &DictymazeParameters.MaxTrackedCells);
            } break;
            case 4: {
                sscanf(SplitTok, "%f", &DictymazeParameters.MinTrackingSize);
            } break;
            case 5: {
                sscanf(SplitTok, "%f", &DictymazeParameters.MinDetectionSize);
            } break;
            case 6: {
                sscanf(SplitTok, "%f", &DictymazeParameters.CellDirection);
            } break;
            case 7: {
                sscanf(SplitTok, "%f", &DictymazeParameters.MaxEstimateDistanceSq);
            } break;
        }

        SplitTok = strtok(0, "\n\r");
        ++FieldIndex;
    }

    Dictymaze(&DictymazeParameters);
    
    return 0;
}
