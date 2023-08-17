cmake_minimum_required(VERSION 3.5)
project(sdk_main)

set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_SUPPRESS_REGENERATION true)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    if(UNIX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    endif()
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

FILE(GLOB CYGLIDAR_D_SRC
    ./src/cyglidar_serial.cpp
    ./src/cyglidar_driver.cpp
    ./src/cygparser.c
)

find_package(Boost REQUIRED COMPONENTS)

if(Boost_FOUND)
    if(WIN32)
        include_directories(include ${Boost_INCLUDE_DIRS})

        add_executable(${PROJECT_NAME}
            sdk_main.cpp
            ${CYGLIDAR_D_SRC})

        target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
    elseif(UNIX)
        include_directories(include ${Boost_INCLUDE_DIRS})

        add_executable(${PROJECT_NAME}
            sdk_main.cpp
            ${CYGLIDAR_D_SRC})

        target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
    endif()
else()
    message(STATUS "Failed to Find Boost Library")
endif()
