cmake_minimum_required(VERSION 3.14)
project(robot_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt packages (prioritize Qt5 since it's available)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2 Multimedia WebSockets Network)

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
    include/websocketbridge.h
    include/tsssocketbridge.h
    include/ros2mapbridge.h
    include/mapimageprovider.h
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
