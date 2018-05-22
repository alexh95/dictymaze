# Dictymaze

## Cell Tracker

### Development

#### Windows (7 or later)

##### Setup
1. Download and install [Microsoft Visual Studio](https://www.visualstudio.com/vs/community/) and make sure to install the Visual C++ package.
2. Add to the environment setup scripts to `PATH`, they are located in the Visual Studio instalation folder i.e. `C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build`.
3. Download and install [OpenCV](http1s://opencv.org/releases.html). Add `<opencv location>\opencv\build\x64\vc15\bin` to `PATH`.
4. If necessary, update `win32_build.bat`'s linker options to include the library for the OpenCV version you installed. (For version 3.41 it would look like this `SET LinkerFlags= ... C:\<opencv location>\opencv\build\x64\vc15\lib\opencv_world341d.lib`)

##### Debug Mode

###### Building the application
Before building, start a command line and execute `vcvars64` if you have a 64 bit OS installed or `vcvars32` otherwise. This will add the appropriate compiler to the `PATH`. Run `win32_debug_build.bat` from the command line to perform a debug build.

###### Running the application
This step assumes you performed *Building the application* for the debug build and you have a command prompt already initialized for VC++. For the debug build you may simply start `win32_dictymaze.exe`. However you may run it with Microsoft's Debugger, Visual Studio, by executing `devenv win32_dictymaze.exe` in a command prompt.

###### NOTE:
There is an included environment setup script called `win32_start_env.bat`. This initializez the command prompt running it, builds the application, starts my text editor of choice and starts Visual Studio targeting the executable.

By starting an application (Sublime, a text editor, in this case) from the terminal that also ran `vcvars64`, that application has the same environment variables. This allows it to run `win32_build.bat` using it's internal build system (with the build script included in `sublime/`) without any other external enviroment variable setup.

##### Release Mode

###### Building the application
Similar to building the debug application, run `win32_release_build.bat` instead.

###### Running the application
Run `win32_dictimaze.exe`.
