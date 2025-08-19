# RobotFace-Healthmate-App

A Qt-based application for robot face health monitoring.

## Prerequisites

- CMake 3.16 or higher
- Qt 5.15+ or Qt 6.0+
- C++17 compatible compiler

## Building

### Windows
```cmd
build.bat
```

### Linux/macOS
```bash
mkdir build
cd build
cmake ..
make
```

## Project Structure

```
RobotFace-Healthmate-App/
├── CMakeLists.txt          # CMake configuration
├── main.cpp               # Application entry point
├── mainwindow.cpp         # Main window implementation
├── mainwindow.h           # Main window header
├── mainwindow.ui          # Qt Designer UI file
├── build.bat              # Windows build script
└── README.md              # This file
```

## Features

- **Simple CMake build system**
- **Qt 5/6 Compatibility**
- **Cross-platform**
- **Modern C++17**

## Troubleshooting

If you get "ninja: build stopped: subcommand failed":

1. **Install Qt**: Download from https://www.qt.io/download
2. **Set Qt path**: `cmake .. -DCMAKE_PREFIX_PATH="C:/Qt/6.5.0/msvc2019_64"`
3. **Use different generator**: `cmake .. -G "Visual Studio 17 2022"`

## License

See LICENSE file for details.