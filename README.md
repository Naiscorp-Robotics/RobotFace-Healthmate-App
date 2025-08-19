# Robot Face Healthmate App

A Qt-based desktop application that provides a robot face interface with chat capabilities, video playback, and image viewing features.

## 🎯 Available Versions

This project now includes **TWO versions** of the same application:

### 1. Qt Widgets Version (Traditional Desktop App)
- **File**: `main.cpp`, `mainwindow.h`, `mainwindow.cpp`
- **Build**: `build.bat` or `CMakeLists.txt`
- **Features**: Traditional desktop UI using Qt Widgets

### 2. QML Version (Modern Declarative UI)
- **File**: `main_qml.cpp`, `Main.qml`, `pages/`, `components/`
- **Build**: `build_qml.bat` or `CMakeLists_qml.txt`
- **Features**: Modern declarative UI using QML

## Features

### 🤖 Robot Face Interface
- **Video Display**: Shows a blinking robot face video (blinking_face.mp4) in a dedicated video player
- **Chat Interface**: Send messages to a robot AI via HTTP API
- **Response Viewer**: View robot responses in a dedicated screen
- **Memory Management**: Reset robot memory with a single click

### 📷 Image Viewer
- **Image Display**: View images (currently j97.jpg) in a full-screen viewer
- **Navigation**: Easy navigation between main interface and image viewer
- **Error Handling**: Graceful handling of missing or corrupted image files

### 🎨 Modern UI
- **Dark Theme**: Consistent dark theme throughout the application
- **Responsive Layout**: Adapts to different window sizes
- **Multi-Screen Navigation**: Stacked widget interface for smooth transitions
- **Status Updates**: Real-time status updates for user feedback

## Technical Details

### Architecture
- **Qt Widgets Version**: Built using Qt6/Qt5 Widgets framework
- **QML Version**: Built using Qt6/Qt5 QML and Quick framework
- **Multimedia Support**: Uses Qt Multimedia for video playback
- **Network Communication**: HTTP client for API communication
- **JSON Handling**: Built-in JSON parsing for API responses

### API Integration
- **Base URL**: `http://192.168.1.122:8989`
- **Endpoints**:
  - `POST /qa_bot/send_message` - Send chat messages
  - `POST /qa_bot/reset_memory` - Reset robot memory

### File Structure
```
RobotFace-Healthmate-App/
├── assets/
│   ├── blinking_face.mp4    # Robot face video
│   └── j97.jpg             # Sample image
├── components/              # QML Components
│   ├── InputBox.qml        # Message input component
│   ├── ControlButton.qml   # Button component
│   └── RobotEye.qml        # Robot eye component
├── pages/                   # QML Pages
│   ├── RobotFaceScreen.qml # Main robot face screen
│   ├── ResponseScreen.qml  # Response viewer screen
│   └── ImageScreen.qml     # Image viewer screen
├── main.cpp                 # Qt Widgets entry point
├── main_qml.cpp            # QML entry point
├── mainwindow.h            # Qt Widgets main window header
├── mainwindow.cpp          # Qt Widgets main window implementation
├── mainwindow.ui           # Qt Widgets UI definition
├── Main.qml               # QML main file
├── resources.qrc          # QML resources
├── CMakeLists.txt         # Qt Widgets build configuration
├── CMakeLists_qml.txt     # QML build configuration
├── build.bat              # Qt Widgets build script
├── build_qml.bat          # QML build script
└── README.md              # This file
```

## Building

### Prerequisites
- Qt6 or Qt5 with the following modules:
  - **Qt Widgets Version**: Core, Gui, Widgets, Multimedia, MultimediaWidgets, Network
  - **QML Version**: Core, Gui, Qml, Quick, QuickControls2, Multimedia
- CMake 3.16 or higher
- MinGW or Visual Studio (Windows)

### Build Instructions

#### Qt Widgets Version
1. **Windows**:
   ```batch
   build.bat
   ```

2. **Manual CMake**:
   ```bash
   mkdir build
   cd build
   cmake .. -G "MinGW Makefiles"
   cmake --build . --config Release
   ```

#### QML Version
1. **Windows**:
   ```batch
   build_qml.bat
   ```

2. **Manual CMake**:
   ```bash
   mkdir build_qml
   cd build_qml
   cmake .. -G "MinGW Makefiles" -f ../CMakeLists_qml.txt
   cmake --build . --config Release
   ```

## Usage

### Qt Widgets Version
1. **Launch**: `RobotFace-Healthmate-App.exe`
2. **Main Screen**:
   - Watch the robot face video
   - Type messages in the input field
   - Click "Send" or press Enter to send messages
   - Click "Reset Memory" to clear robot memory
   - Click "📷 Open Image" to view images

### QML Version
1. **Launch**: `RobotFace-Healthmate-App-QML.exe`
2. **Main Screen**:
   - Watch the robot face video
   - Type messages in the input field
   - Click "Send" or press Enter to send messages
   - Click "Reset Memory" to clear robot memory
   - Click "📷 Open Image" to view images

Both versions provide identical functionality with different UI frameworks.

## Network Configuration

The application is configured to connect to a local server at `192.168.1.122:8989`. To change the server address:

1. **Qt Widgets Version**: Edit `mainwindow.cpp`, find `sendNetworkRequest` function
2. **QML Version**: Edit `pages/RobotFaceScreen.qml`, find the XMLHttpRequest URLs

## Error Handling

The application includes comprehensive error handling for:
- Network connection failures
- Invalid API responses
- Missing media files
- File loading errors

## Dependencies

### Qt Widgets Version
- **Qt Core**: Core non-GUI functionality
- **Qt Gui**: GUI components and window system integration
- **Qt Widgets**: UI components
- **Qt Multimedia**: Audio/video playback
- **Qt MultimediaWidgets**: Multimedia widgets
- **Qt Network**: Network programming support

### QML Version
- **Qt Core**: Core non-GUI functionality
- **Qt Gui**: GUI components and window system integration
- **Qt Qml**: QML engine and language support
- **Qt Quick**: QML UI framework
- **Qt QuickControls2**: QML UI controls
- **Qt Multimedia**: Audio/video playback

## Migration Notes

This project contains both the original QML interface (migrated from RobotFaceByCurosr) and a Qt Widgets version. The following features were preserved in both versions:

- ✅ Video playback functionality
- ✅ Chat interface with API integration
- ✅ Image viewing capabilities
- ✅ Multi-screen navigation
- ✅ Dark theme styling
- ✅ Error handling and status updates
- ✅ Memory reset functionality

### QML Components
- **InputBox.qml**: Reusable message input component
- **ControlButton.qml**: Reusable button component
- **RobotEye.qml**: Robot eye visualization component

### QML Pages
- **RobotFaceScreen.qml**: Main interface with video player and chat
- **ResponseScreen.qml**: Response viewer with back navigation
- **ImageScreen.qml**: Image viewer with back navigation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
=======
## License

See LICENSE file for details.
