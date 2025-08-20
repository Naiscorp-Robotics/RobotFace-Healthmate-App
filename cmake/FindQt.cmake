# Custom Qt finder module
# This module provides better Qt detection and configuration

# Common Qt installation paths on Windows
set(QT_POSSIBLE_PATHS
    "C:/Qt"
    "C:/Qt6"
    "C:/Qt5"
    "D:/Qt"
    "D:/Qt6"
    "D:/Qt5"
    "$ENV{QTDIR}"
    "$ENV{QT_DIR}"
)

# Try to find Qt6 first, then fallback to Qt5
find_package(Qt6 QUIET COMPONENTS Core Gui Widgets LinguistTools)
if(Qt6_FOUND)
    set(QT_VERSION 6)
    set(QT_FOUND TRUE)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
else()
    find_package(Qt5 QUIET COMPONENTS Core Gui Widgets LinguistTools)
    if(Qt5_FOUND)
        set(QT_VERSION 5)
        set(QT_FOUND TRUE)
        message(STATUS "Found Qt5: ${Qt5_VERSION}")
    else()
        # Try to find Qt in common installation paths
        foreach(QT_PATH ${QT_POSSIBLE_PATHS})
            if(EXISTS ${QT_PATH})
                message(STATUS "Found Qt installation at: ${QT_PATH}")
                list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})
                break()
            endif()
        endforeach()
        
        # Try again with updated prefix path
        find_package(Qt6 QUIET COMPONENTS Core Gui Widgets LinguistTools)
        if(Qt6_FOUND)
            set(QT_VERSION 6)
            set(QT_FOUND TRUE)
            message(STATUS "Found Qt6: ${Qt6_VERSION}")
        else()
            find_package(Qt5 QUIET COMPONENTS Core Gui Widgets LinguistTools)
            if(Qt5_FOUND)
                set(QT_VERSION 5)
                set(QT_FOUND TRUE)
                message(STATUS "Found Qt5: ${Qt5_VERSION}")
            else()
                message(FATAL_ERROR 
                    "Neither Qt5 nor Qt6 found. Please install Qt development libraries.\n"
                    "Common installation paths checked: ${QT_POSSIBLE_PATHS}\n"
                    "You can set CMAKE_PREFIX_PATH to your Qt installation directory:\n"
                    "cmake .. -DCMAKE_PREFIX_PATH=\"C:/Qt/6.5.0/msvc2019_64\""
                )
            endif()
        endif()
    endif()
endif()

# Set Qt-specific variables
if(QT_FOUND)
    set(QT_CORE_LIBRARY Qt${QT_VERSION}::Core)
    set(QT_GUI_LIBRARY Qt${QT_VERSION}::Gui)
    set(QT_WIDGETS_LIBRARY Qt${QT_VERSION}::Widgets)
    set(QT_LINGUISTTOOLS_LIBRARY Qt${QT_VERSION}::LinguistTools)
    
    # Enable Qt automoc, autouic, and autorcc
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)
endif()
