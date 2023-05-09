cmake_minimum_required(VERSION 2.8.3)

set(PROJECTNAME "cyglidar_d1_ros1")
project(${PROJECTNAME} CXX C)

set(extra_flags -std=c++11 -Werror -Wall -Wextra)
set(CMAKE_CXX_FLGAS "-std=c++11 -03")
set(CMAKE_C_LINK_EXECUTABLE ${CMAKE_CXX_LINK_EXECUTABLE})

add_compile_options(-std=c++14)
add_definitions(${COMPILE_FLAGS} ${PCL_DEFINITIONS})

set(CYGLIDAR_SDK_PATH "./sdk/")
FILE(GLOB CYGLIDAR_D_SRC
    "${CYGLIDAR_SDK_PATH}/src/*.cpp"
    "${CYGLIDAR_SDK_PATH}/src/*.c"
)

set(CYGLIDAR_ROS_PATH "${CYGLIDAR_SDK_PATH}/D1_ROS1/")
FILE(GLOB CYGLIDAR_ROS_SRC
    "${CYGLIDAR_ROS_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.2 REQUIRED)

include_directories(
    ${CYGLIDAR_SDK_PATH}/include
    ${CYGLIDAR_SDK_PATH}/src
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CYGLIDAR_ROS_PATH}/include
)

catkin_package()

add_library(${PROJECT_NAME} ${CYGLIDAR_SDK_PATH}/src/cygparser.c)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(cyglidar_publisher
    ${CYGLIDAR_ROS_PATH}/cyglidar_publisher.cpp
    ${CYGLIDAR_D_SRC}
    ${CYGLIDAR_ROS_SRC}
)

add_dependencies(cyglidar_publisher ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(cyglidar_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

install(TARGETS cyglidar_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
