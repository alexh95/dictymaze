@ECHO off 

SET CompilerFlags= /MTd /nologo /wd4312 /EHa- /EHsc- /Z7 /I C:\Libs\opencv\build\include
SET LinkerFlags= /incremental:no /opt:ref C:\Libs\opencv\build\x64\vc15\lib\opencv_world341d.lib

IF NOT EXIST build MKDIR build
PUSHD build

DEL *.pdb 1> NUL 2> NUL
cl %CompilerFlags% ..\src\win32_dictymaze.cpp /link %LinkerFlags%

POPD
