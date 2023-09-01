cmake_minimum_required(VERSION 3.5)

project(cyglidar_d1_ros2)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CYGLIDAR_SDK_PATH "./sdk/")
FILE(GLOB CYGLIDAR_D_SRC
    "${CYGLIDAR_SDK_PATH}/src/*.cpp"
    "${CYGLIDAR_SDK_PATH}/src/*.c"
)

set(CYGLIDAR_ROS_PATH "./D1_ROS2/")
FILE(GLOB CYGLIDAR_ROS_SRC
    "${CYGLIDAR_ROS_PATH}/src/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

include_directories(
    ${CYGLIDAR_SDK_PATH}/include
    ${CYGLIDAR_ROS_PATH}/include
    ${pcl_conversions_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${CYGLIDAR_SDK_PATH}/src/cygparser.c)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(cyglidar_d1_publisher
    ${CYGLIDAR_ROS_PATH}/cyglidar_publisher.cpp
    ${CYGLIDAR_ROS_SRC}
    ${CYGLIDAR_D_SRC}
)

ament_target_dependencies(cyglidar_d1_publisher
    rclcpp
    std_msgs
    sensor_msgs
    Boost
    PCL
)

install(TARGETS cyglidar_d1_publisher
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CYGLIDAR_SDK_PATH}/include ${CYGLIDAR_ROS_PATH}/include
    DESTINATION include)

install(DIRECTORY launch rviz scripts
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(Boost)
ament_export_dependencies(PCL)
ament_package()
