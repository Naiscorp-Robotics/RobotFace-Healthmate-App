cmake_minimum_required(VERSION 3.16)

project(RobotFace-Healthmate-App VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    Qml
    Quick
    QuickControls2
    Multimedia
    MultimediaWidgets
    Network
    WebSockets
)
if(NOT Qt6_FOUND)
    find_package(Qt5 REQUIRED COMPONENTS
        Core
        Gui
        Widgets
        Qml
        Quick
        QuickControls2
        Multimedia
        MultimediaWidgets
        Network
        WebSockets
    )
endif()

# Find ALSA library (optional for platforms that support it)
find_package(ALSA)

# Enable Qt automoc, autouic, and autorcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Kiểm tra và thêm các file nguồn nếu chúng tồn tại
function(add_files_if_exist variable)
    set(files)
    foreach(file ${ARGN})
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${file})
            list(APPEND files ${file})
        else()
            message(WARNING "File ${file} not found, skipping")
        endif()
    endforeach()
    set(${variable} ${files} PARENT_SCOPE)
endfunction()

# Source files - chỉ thêm nếu file tồn tại
add_files_if_exist(SOURCES
    main.cpp
    mainwindow.cpp
    audiomanager.cpp
    websocketbridge.cpp
)

# Header files - chỉ thêm nếu file tồn tại
add_files_if_exist(HEADERS
    mainwindow.h
    audiomanager.h
    websocketbridge.h
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
    resources.qrc
)

# Create executable với chỉ những file tồn tại
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${FORMS}
    ${RESOURCES}
)

# Link all libraries
if(Qt6_FOUND)
    target_link_libraries(${PROJECT_NAME}
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Qml
        Qt6::Quick
        Qt6::QuickControls2
        Qt6::Multimedia
        Qt6::MultimediaWidgets
        Qt6::Network
        Qt6::WebSockets
    )
else()
    target_link_libraries(${PROJECT_NAME}
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Qml
        Qt5::Quick
        Qt5::QuickControls2
        Qt5::Multimedia
        Qt5::MultimediaWidgets
        Qt5::Network
        Qt5::WebSockets
    )
endif()

# Thêm ALSA nếu tìm thấy
if(ALSA_FOUND)
    target_link_libraries(${PROJECT_NAME} ${ALSA_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${ALSA_INCLUDE_DIRS})
else()
    message(WARNING "ALSA library not found, audio features may be limited")
endif()

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Copy assets, pages, components and QML files to build directory
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pages)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pages DESTINATION ${CMAKE_BINARY_DIR})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/components)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/components DESTINATION ${CMAKE_BINARY_DIR})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Main.qml)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Main.qml DESTINATION ${CMAKE_BINARY_DIR})
endif()

# Set application properties for macOS
if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.healthmate.robotface"
        MACOSX_BUNDLE_BUNDLE_NAME "RobotFace Healthmate"
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
    )
endif()

# Add resource file that includes QML files (nếu tồn tại)
foreach(qml_file ${QML_FILES})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${qml_file})
        list(APPEND QML_RESOURCE_FILES ${qml_file})
    endif()
endforeach()

if(QML_RESOURCE_FILES)
    qt_add_resources(${PROJECT_NAME} "qml_resources"
        PREFIX "/"
        FILES ${QML_RESOURCE_FILES}
    )
endif()

# Add install targets for Linux
if(UNIX AND NOT APPLE)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/assets)
        install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
    endif()
endif()

# Hiển thị thông tin về các file được thêm vào
message(STATUS "Source files: ${SOURCES}")
message(STATUS "Header files: ${HEADERS}")
message(STATUS "Form files: ${FORMS}")
