cmake_minimum_required(VERSION 3.8)
project(ros2_qt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

link_directories(/opt/ros/humble/lib)

add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    rclcpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
)
