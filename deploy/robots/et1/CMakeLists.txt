cmake_minimum_required(VERSION 3.12)
project(et1_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

if(DEFINED ENV{UNITREE_SDK2_PATH})
    set(UNITREE_SDK2_PATH $ENV{UNITREE_SDK2_PATH})
    if(EXISTS "${UNITREE_SDK2_PATH}/lib/cmake/unitree_sdk2")
        list(APPEND CMAKE_PREFIX_PATH "${UNITREE_SDK2_PATH}")
    endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")

find_package(unitree_sdk2 REQUIRED CONFIG)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" TARGET_PROCESSOR)
set(THIRDPARTY_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty")

if(NOT DEFINED ONNXRUNTIME_ROOT)
    if(TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
        set(ONNXRUNTIME_ROOT "${THIRDPARTY_DIR}/onnxruntime-linux-x64-1.22.0")
    elseif(TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
        set(ONNXRUNTIME_ROOT "${THIRDPARTY_DIR}/onnxruntime-linux-aarch64-1.26.0")
    elseif(TARGET_PROCESSOR MATCHES "^arm")
        set(ONNXRUNTIME_ROOT "${THIRDPARTY_DIR}/onnxruntime-linux-arm-1.22.0")
    else()
        message(FATAL_ERROR
            "Unsupported target processor: ${CMAKE_SYSTEM_PROCESSOR}. "
            "Set ONNXRUNTIME_ROOT to the target ONNX Runtime directory.")
    endif()
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS "${ONNXRUNTIME_ROOT}/include"
    NO_DEFAULT_PATH
    REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime libonnxruntime.so.1.26.0 libonnxruntime.so.1.22.0
    PATHS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
    NO_DEFAULT_PATH
    REQUIRED
)

message(STATUS "Target processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "ONNX Runtime root: ${ONNXRUNTIME_ROOT}")

set(UNITREE_SDK2_RUNTIME_DIR "/opt/unitree_robotics/lib")
if(DEFINED UNITREE_SDK2_PATH AND EXISTS "${UNITREE_SDK2_PATH}/lib")
    set(UNITREE_SDK2_RUNTIME_DIR "${UNITREE_SDK2_PATH}/lib")
endif()
set(CMAKE_BUILD_RPATH "${UNITREE_SDK2_RUNTIME_DIR}")
set(CMAKE_INSTALL_RPATH "${UNITREE_SDK2_RUNTIME_DIR}")

include_directories(
  /usr/include/eigen3
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 rt pthread
  Boost::program_options yaml-cpp fmt
  ${ONNXRUNTIME_LIBRARY}
  ${ZMQ_LIBRARIES}
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(et1_ctrl main.cpp)
