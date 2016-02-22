cmake_minimum_required(VERSION 2.8.7)
project(imu_driver_odroid)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Boost REQUIRED COMPONENTS thread system chrono)


# SSE flags
#add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
#set(CMAKE_CXX_FLAGS
#   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -O2 -march=native  -std=c++0x"
#)

# NEON flags
add_definitions("-DENABLE_NEON")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -O2 -march=native -mfpu=neon -std=c++0x"
) 

# Set source files
set(imu_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/ImuDriver.cpp
)
set(SOURCE_FILES
  ${imu_SOURCE_FILES}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
)


# build shared library.
add_library(imu_lib SHARED ${SOURCE_FILES})

target_link_libraries(imu_lib ${Boost_LIBRARIES})


# build test program
add_executable(imu_test src/test/main_test.cpp)
target_link_libraries(imu_test imu_lib)
