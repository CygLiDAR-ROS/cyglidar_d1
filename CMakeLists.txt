cmake_minimum_required(VERSION 3.5)

project(cyglidar_d1)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CYGLIDAR_SDK_PATH "./sdk/")

FILE(GLOB CYGLIDAR_D_SRC
    "${CYGLIDAR_SDK_PATH}/src/*.cpp"
    "${CYGLIDAR_SDK_PATH}/src/*.c"
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

include_directories(${CYGLIDAR_SDK_PATH}/include)

add_library(${PROJECT_NAME} ${CYGLIDAR_SDK_PATH}/src/cygparser.c)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(CyglidarNode
    src/cyglidar_node.cpp
    ${CYGLIDAR_D_SRC}
)

ament_target_dependencies(CyglidarNode
    rclcpp
    std_msgs
    sensor_msgs
    Boost
    PCL
)

install(TARGETS CyglidarNode
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CYGLIDAR_SDK_PATH}/include/
    DESTINATION include)

install(
    DIRECTORY launch rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(Boost)
ament_export_dependencies(PCL)
ament_package()
