cmake_minimum_required(VERSION 3.14)
project(robot_app)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages (prioritize Qt5 since it's available)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Multimedia WebSockets Network)

# Find ALSA library
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
    src/ros2mapbridge.cpp
    src/mapimageprovider.cpp
    src/audiomanager.cpp
    src/filehelper.cpp
    include/websocketbridge.h
    include/tsssocketbridge.h
    include/ros2mapbridge.h
    include/mapimageprovider.h
    include/audiomanager.h
    include/filehelper.h
)

# Form files - only add if file exists
set(FORMS "")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.ui")
    list(APPEND FORMS mainwindow.ui)
endif()

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

# Link Qt5 libraries and ALSA
target_link_libraries(${PROJECT_NAME} 
    Qt5::Core 
    Qt5::Gui 
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
    Qt5::Multimedia
    Qt5::Network
    Qt5::WebSockets
    ${ALSA_LIBRARIES}
)

# Link ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
)

# Include ALSA header directories
target_include_directories(${PROJECT_NAME} PRIVATE ${ALSA_INCLUDE_DIRS})

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Copy assets to build directory
file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/pages DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/components DESTINATION ${CMAKE_BINARY_DIR})
file(COPY forms/Main.qml DESTINATION ${CMAKE_BINARY_DIR})

# Create package.xml if it doesn't exist
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" 
"<?xml version=\"1.0\"?>
<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>
<package format=\"3\">
  <name>robot_app</name>
  <version>0.1.0</version>
  <description>RobotFace Healthmate App with ROS2 Map Integration</description>
  <maintainer email=\"you@example.com\">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>std_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>")
endif()

ament_package()
