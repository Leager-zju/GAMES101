cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.hpp")
add_executable(RayTracing ${SRCS})
target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -g -O3)