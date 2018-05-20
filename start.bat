call vcvars64
call build.bat
start subl .
pushd build
devenv win32_dictymaze.exe
popd