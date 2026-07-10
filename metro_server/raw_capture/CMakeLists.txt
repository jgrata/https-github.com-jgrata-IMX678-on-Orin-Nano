cmake_minimum_required(VERSION 3.10)
project(raw_capture CXX)
set(CMAKE_CXX_STANDARD 14)

set(ARGUS_INC /usr/src/jetson_multimedia_api/argus/include)

find_path(NVBUF_INC nvbufsurface.h
    PATHS /usr/src/jetson_multimedia_api/include
          /usr/include/aarch64-linux-gnu /usr/include REQUIRED)

find_path(CUDA_INC cuda.h
    PATHS /usr/local/cuda/include /usr/include REQUIRED)

find_library(CUDA_LIB cuda
    PATHS /usr/lib/aarch64-linux-gnu/nvidia
          /usr/lib/aarch64-linux-gnu
          /usr/local/cuda/lib64 REQUIRED)

find_library(NVARGUS_LIB nvargus_socketclient
    PATHS /usr/lib/aarch64-linux-gnu/nvidia REQUIRED)

find_library(NVBUF_LIB nvbufsurface
    PATHS /usr/lib/aarch64-linux-gnu/tegra
          /usr/lib/aarch64-linux-gnu REQUIRED)

find_library(EGL_LIB EGL
    PATHS /usr/lib/aarch64-linux-gnu REQUIRED)

message(STATUS "CUDA_INC    = ${CUDA_INC}")
message(STATUS "CUDA_LIB    = ${CUDA_LIB}")

add_executable(raw_capture raw_capture.cpp)

target_include_directories(raw_capture PRIVATE
    ${ARGUS_INC} ${ARGUS_INC}/Argus ${ARGUS_INC}/EGLStream
    ${NVBUF_INC} ${CUDA_INC})

target_link_libraries(raw_capture
    ${NVARGUS_LIB} ${NVBUF_LIB} ${EGL_LIB} ${CUDA_LIB} pthread)
