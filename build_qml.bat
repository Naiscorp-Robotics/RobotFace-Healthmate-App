@echo off
echo Building RobotFace-Healthmate-App QML Version...

if exist "build_qml" rmdir /s /q build_qml
mkdir build_qml
cd build_qml

echo Configuring with CMake for QML version...
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -f ../CMakeLists_qml.txt

if %errorlevel% neq 0 (
    echo Configuration failed!
    pause
    exit /b 1
)

echo Building QML project...
cmake --build . --config Release

if %errorlevel% neq 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo QML Build completed successfully!
echo Executable: build_qml\RobotFace-Healthmate-App-QML.exe
pause
