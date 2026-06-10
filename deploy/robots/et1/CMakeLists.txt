cmake_minimum_required(VERSION 3.12)
project(et1_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" TARGET_PROCESSOR)
set(THIRDPARTY_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty")

if(DEFINED ENV{UNITREE_SDK2_PATH})
    set(UNITREE_SDK2_PATH "$ENV{UNITREE_SDK2_PATH}")
elseif(EXISTS "${PROJECT_SOURCE_DIR}/../../../../unitree_sdk2")
    set(UNITREE_SDK2_PATH "${PROJECT_SOURCE_DIR}/../../../../unitree_sdk2")
elseif(EXISTS "/root/unitree_sdk2")
    set(UNITREE_SDK2_PATH "/root/unitree_sdk2")
elseif(EXISTS "/root/penggang/unitree_sdk2")
    set(UNITREE_SDK2_PATH "/root/penggang/unitree_sdk2")
elseif(EXISTS "/opt/unitree_robotics")
    set(UNITREE_SDK2_PATH "/opt/unitree_robotics")
endif()

set(UNITREE_SDK2_LINK_LIBS unitree_sdk2)
set(UNITREE_SDK2_RUNTIME_DIR "/opt/unitree_robotics/lib")

set(UNITREE_SDK2_LOCAL_AVAILABLE FALSE)
if(DEFINED UNITREE_SDK2_PATH)
    set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_PATH}/lib/${CMAKE_SYSTEM_PROCESSOR}")
    set(UNITREE_SDK2_THIRDPARTY_LIB_DIR "${UNITREE_SDK2_PATH}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
    set(UNITREE_SDK2_LIB "${UNITREE_SDK2_LIB_DIR}/libunitree_sdk2.a")
    set(UNITREE_SDK2_DDSC_LIB "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}/libddsc.so")
    set(UNITREE_SDK2_DDSCXX_LIB "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}/libddscxx.so")
    if(EXISTS "${UNITREE_SDK2_LIB}" AND EXISTS "${UNITREE_SDK2_DDSC_LIB}" AND EXISTS "${UNITREE_SDK2_DDSCXX_LIB}")
        set(UNITREE_SDK2_LOCAL_AVAILABLE TRUE)
    endif()
endif()

if(UNITREE_SDK2_LOCAL_AVAILABLE)
    add_library(unitree_sdk2 STATIC IMPORTED)
    set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION "${UNITREE_SDK2_LIB}")
    list(APPEND UNITREE_SDK2_LINK_LIBS "${UNITREE_SDK2_DDSC_LIB}" "${UNITREE_SDK2_DDSCXX_LIB}")
    set(UNITREE_SDK2_RUNTIME_DIR "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}")
    message(STATUS "Using local Unitree SDK library: ${UNITREE_SDK2_LIB}")
else()
    if(DEFINED UNITREE_SDK2_PATH)
        list(APPEND CMAKE_PREFIX_PATH
            "${UNITREE_SDK2_PATH}"
            "${UNITREE_SDK2_PATH}/lib/cmake/unitree_sdk2"
        )
    endif()
    list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")
    find_package(unitree_sdk2 REQUIRED CONFIG)

    message(STATUS "unitree_sdk2 package found")
    if(DEFINED UNITREE_SDK2_PATH AND EXISTS "${UNITREE_SDK2_PATH}/lib")
        set(UNITREE_SDK2_RUNTIME_DIR "${UNITREE_SDK2_PATH}/lib")
    endif()
endif()

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

set(CMAKE_BUILD_RPATH "${UNITREE_SDK2_RUNTIME_DIR};${ONNXRUNTIME_ROOT}/lib;${ONNXRUNTIME_ROOT}/lib64")
set(CMAKE_INSTALL_RPATH "${CMAKE_BUILD_RPATH}")

include_directories(
  /usr/include/eigen3
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../include/
  ${UNITREE_SDK2_PATH}/include
  ${UNITREE_SDK2_PATH}/thirdparty/include
  ${UNITREE_SDK2_PATH}/thirdparty/include/ddscxx
)

link_libraries(
  ${UNITREE_SDK2_LINK_LIBS} rt pthread
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
