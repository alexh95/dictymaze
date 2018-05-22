CALL vcvars64
CD
CALL win32_debug_build.bat
START subl .
PUSHD build
devenv win32_dictymaze.exe
POPD
