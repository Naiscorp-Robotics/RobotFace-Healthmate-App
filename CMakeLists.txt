cmake_minimum_required(VERSION 3.14)
project(QtWebRTCClient)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages
find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Multimedia WebSockets Network)
if(NOT Qt6_FOUND)
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Multimedia WebSockets Network)
endif()

# Enable Qt automoc, autouic, and autorcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Source files
set(SOURCES
    main.cpp
    websocketbridge.cpp
    websocketbridge.h
)

set(RESOURCES
    resources.qrc
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${RESOURCES})

# Link Qt libraries
if(Qt6_FOUND)
    target_link_libraries(${PROJECT_NAME} 
        Qt6::Core 
        Qt6::Gui 
        Qt6::Qml
        Qt6::Quick
        Qt6::QuickControls2
        Qt6::Multimedia
        Qt6::Network
        Qt6::WebSockets
    )
else()
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
endif()

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Copy assets to build directory
file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})
file(COPY pages DESTINATION ${CMAKE_BINARY_DIR})
file(COPY components DESTINATION ${CMAKE_BINARY_DIR})
file(COPY Main.qml DESTINATION ${CMAKE_BINARY_DIR})
