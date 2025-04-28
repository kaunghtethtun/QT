cmake_minimum_required(VERSION 3.5)

project(Testing VERSION 0.1 LANGUAGES CXX)

# Enable Qt's automatic MOC and UIC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -- Find Qt libraries
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# -- Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

# Create executable based on the Qt version
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Testing
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(Testing
        ${PROJECT_SOURCES}
    )
endif()

# Link Qt libraries
target_link_libraries(Testing
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
)

# --- ROS2 dependencies using ament_target_dependencies (no need for target_link_libraries for ROS2 packages)
ament_target_dependencies(Testing
    rclcpp
    std_msgs
)

# Set target properties for bundle on macOS and executable on Windows
set_target_properties(Testing PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Finalize executable for Qt6 (if using Qt6)
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Testing)
endif()

ament_package()
