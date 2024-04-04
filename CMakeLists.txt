cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(Eigen3_DIR "D:/MSYS2/MSYS2/usr/include/eigen3/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 Not Found!")
endif(NOT Eigen3_FOUND)
message(STATUS "Eigen3 library status:")
message(STATUS "  version: ${Eigen3}")
message(STATUS "  libraries: ${Eigen3}")
message(STATUS "  include path: ${Eigen3_INCLUDE_DIRS}")
include_directories("D:/MSYS2/MSYS2/usr/include/eigen3/include")

set(OpenCV_DIR "D:/MSYS2/MSYS2/usr/include/opencv")
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV Not Found!")
endif(NOT OpenCV_FOUND)
message(STATUS "OpenCV library status:")
message(STATUS "  version: ${OpenCV_VERSION}")
message(STATUS "  libraries: ${OpenCV_LIBS}")
message(STATUS "  include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++17 ${OpenCV_CXX_LAGS} -Wno-dev"
)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.hpp")
add_executable(Rasterizer ${SRCS})
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
