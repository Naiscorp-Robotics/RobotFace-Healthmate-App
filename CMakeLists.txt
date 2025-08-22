cmake_minimum_required(VERSION 3.16)
project(trung VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Multimedia Network WebSockets)
qt_standard_project_setup()

set(SOURCES
    main.cpp
    mainwindow.cpp
    AudioRecorder.cpp
    StreamUploader.cpp
)

set(HEADERS
    mainwindow.h
    AudioRecorder.h
    StreamUploader.h
)

set(FORMS
    mainwindow.ui
)

add_executable(trung
    ${SOURCES}
    ${HEADERS}
    ${FORMS}
)

target_link_libraries(trung PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::Multimedia
    Qt6::Network
    Qt6::WebSockets
)

target_include_directories(trung
    PRIVATE
    "Header Files"
)
