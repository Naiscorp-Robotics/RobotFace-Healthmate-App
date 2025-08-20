cmake_minimum_required(VERSION 3.14)
project(robot_app)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages (prioritize Qt5 since it's available)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Multimedia WebSockets Network)

# Set Qt version for conditional compilation
set(QT_VERSION_MAJOR 5)

# Enable Qt automoc and autorcc for QML
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/websocketbridge.cpp
    src/tsssocketbridge.cpp
    src/audiomanager.cpp
    include/websocketbridge.h
    include/tsssocketbridge.h
    include/audiomanager.h
)

# Form files - chỉ thêm nếu file tồn tại
add_files_if_exist(FORMS
    mainwindow.ui
)

# List QML files for reference
set(QML_FILES
    Main.qml
    components/ControlButton.qml
    components/InputBox.qml
    components/RobotEye.qml
    components/StatusIndicator.qml
    components/RecordButton.qml
    components/PlayButton.qml
    components/AudioLevelMeter.qml
    components/ErrorPopup.qml
    components/audioUi.qml
    pages/ImageScreen.qml
    pages/ResponseScreen.qml
    pages/RobotFaceScreen.qml
)

# Asset files
set(ASSETS
    assets/blinking_face.mp4
    assets/j97.jpg
)

# Resource files
set(RESOURCES
    forms/resources.qrc
)

# Create executable với chỉ những file tồn tại
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${FORMS}
    ${RESOURCES}
)

# Link Qt5 libraries
target_link_libraries(${PROJECT_NAME} 
    Qt5::Core 
    Qt5::Gui 
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
    Qt5::Multimedia
    Qt5::Network
    Qt5::WebSockets
)

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Copy assets to build directory
file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/pages DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/components DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/Main.qml DESTINATION ${CMAKE_BINARY_DIR})
