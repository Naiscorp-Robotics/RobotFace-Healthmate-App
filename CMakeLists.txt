cmake_minimum_required(VERSION 3.16)

project(RobotFace-Healthmate-App VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Multimedia MultimediaWidgets Network)
if(NOT Qt6_FOUND)
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Multimedia MultimediaWidgets Network)
endif()

# Enable Qt automoc, autouic, and autorcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Source files
set(SOURCES
    main.cpp
    mainwindow.cpp
)

set(QML_FILES
    Main.qml
    components/ControlButton.qml
    components/InputBox.qml
    components/RobotEye.qml
    pages/ImageScreen.qml
    pages/ResponseScreen.qml
    pages/RobotFaceScreen.qml
)

set(HEADERS
    mainwindow.h
)

set(FORMS
    mainwindow.ui
)

set(ASSETS
    assets/blinking_face.mp4
    assets/j97.jpg
)

# Create executable
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${FORMS}
    ${QML_FILES}
    ${ASSETS}
)


# Link Qt libraries
if(Qt6_FOUND)
    target_link_libraries(${PROJECT_NAME} 
        Qt6::Core 
        Qt6::Gui 
        Qt6::Widgets
        Qt6::Multimedia
        Qt6::MultimediaWidgets
        Qt6::Network
    )
else()
    target_link_libraries(${PROJECT_NAME} 
        Qt5::Core 
        Qt5::Gui 
        Qt5::Widgets
        Qt5::Multimedia
        Qt5::MultimediaWidgets
        Qt5::Network
    )
endif()

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Copy assets to build directory
file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})


