cmake_minimum_required(VERSION 3.14)
project(robot_app)

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
    include/websocketbridge.h
    include/tsssocketbridge.h
)

set(RESOURCES
    forms/resources.qrc
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${RESOURCES})

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
