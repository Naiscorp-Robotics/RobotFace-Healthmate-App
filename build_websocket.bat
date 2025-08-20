@echo off
echo Building RobotFace Healthmate App with integrated WebSocket support...

REM Create build directory if it doesn't exist
if not exist "build" mkdir build
cd build

REM Configure with CMake
cmake .. -G "MinGW Makefiles"

REM Build the project
cmake --build .

REM Run the application
echo.
echo Running RobotFace Healthmate App with WebSocket...
QtWebRTCClient.exe

pause
