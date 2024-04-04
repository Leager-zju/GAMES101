cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(Eigen3_DIR "D:/MSYS2/MSYS2/usr/include/eigen3") # change to your/path/to/eigen3
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_DIR}/include)

set(OpenCV_DIR "D:/MSYS2/MSYS2/usr/include/opencv") # change to your/path/to/opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIR}/include)

set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++17 ${OpenCV_CXX_LAGS}"
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.hpp")
add_executable(Rasterizer ${SRCS})
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
