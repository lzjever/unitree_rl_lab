cmake_minimum_required(VERSION 3.16)

project(agentic_et1_tracker LANGUAGES CXX)

option(AGENTIC_ET1_BUILD_TESTS "Build pure core unit tests" ON)
option(AGENTIC_ET1_BUILD_ROBOT "Build robot integration target" ON)
option(AGENTIC_ET1_BUILD_ONNX "Build ONNX policy runtime target" ON)
option(AGENTIC_ET1_BUILD_PERF_SMOKE "Build opt-in performance smoke tests" OFF)

add_library(agentic_et1_tracker_core
  src/command_mailbox.cpp
  src/id_generator.cpp
  src/motion_queue.cpp
  src/recent_ring.cpp
  src/status.cpp
  src/tracker_controller.cpp
  src/types.cpp
)

target_compile_features(agentic_et1_tracker_core PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(agentic_et1_tracker_robot_core
  src/robot_io.cpp
)

target_compile_features(agentic_et1_tracker_robot_core PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_robot_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_robot_core
  PUBLIC
    agentic_et1_tracker_core
)

add_library(agentic_et1_tracker_trk
  src/trk_loader.cpp
  src/trk_reference_alignment.cpp
  src/trk_scanner.cpp
  src/trk_synthetic_transition.cpp
  src/trk_validator.cpp
)

target_compile_features(agentic_et1_tracker_trk PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_trk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_trk
  PUBLIC
    agentic_et1_tracker_core
)

add_library(agentic_et1_tracker_loco_upper
  src/loco_lower_policy.cpp
  src/loco_upper_lowcmd_composer.cpp
  src/loco_upper_plan_compiler.cpp
  src/loco_upper_precheck.cpp
  src/loco_upper_planner.cpp
  src/loco_upper_validator.cpp
)

target_compile_features(agentic_et1_tracker_loco_upper PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_loco_upper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_loco_upper
  PUBLIC
    agentic_et1_tracker_trk
  PRIVATE
    ${AGENTIC_ET1_YAML_TARGET}
)

add_library(agentic_et1_tracker_api
  src/api_service.cpp
  src/json_codec.cpp
)

target_compile_features(agentic_et1_tracker_api PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_api
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include
)
target_link_libraries(agentic_et1_tracker_api
  PUBLIC
    agentic_et1_tracker_core
)

add_library(agentic_et1_tracker_reference
  src/reference_frame.cpp
  src/reference_frame_json.cpp
  src/reference_frame_store.cpp
)

target_compile_features(agentic_et1_tracker_reference PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_reference
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include
)
target_link_libraries(agentic_et1_tracker_reference
  PUBLIC
    agentic_et1_tracker_trk
)

add_library(agentic_et1_tracker_runtime
  src/runtime_bridge.cpp
  src/runtime_control_loop.cpp
  src/runtime_status_store.cpp
)

target_compile_features(agentic_et1_tracker_runtime PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_runtime
  PUBLIC
    agentic_et1_tracker_api
    agentic_et1_tracker_loco_upper
    agentic_et1_tracker_reference
    agentic_et1_tracker_trk
)

find_package(Threads REQUIRED)

find_package(yaml-cpp CONFIG REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(AGENTIC_ET1_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(AGENTIC_ET1_YAML_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp target not found")
endif()

target_link_libraries(agentic_et1_tracker_loco_upper
  PRIVATE
    ${AGENTIC_ET1_YAML_TARGET}
)

add_library(agentic_et1_tracker_policy_math
  src/observation_builder.cpp
  src/policy_math.cpp
)

target_compile_features(agentic_et1_tracker_policy_math PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_policy_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_policy_math
  PUBLIC
    agentic_et1_tracker_trk
)

add_library(agentic_et1_tracker_policy_io_contract
  src/policy_io_contract.cpp
)

target_compile_features(agentic_et1_tracker_policy_io_contract PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_policy_io_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_policy_io_contract
  PUBLIC
    agentic_et1_tracker_policy_math
)

add_library(agentic_et1_tracker_control
  src/fixstand.cpp
  src/passive.cpp
)

target_compile_features(agentic_et1_tracker_control PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_control
  PUBLIC
    agentic_et1_tracker_robot_core
  PRIVATE
    ${AGENTIC_ET1_YAML_TARGET}
)

add_library(agentic_et1_tracker_velocity_policy
  src/velocity_deploy_config.cpp
  src/velocity_policy_runner.cpp
)

target_compile_features(agentic_et1_tracker_velocity_policy PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_velocity_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_velocity_policy
  PUBLIC
    agentic_et1_tracker_policy_math
    agentic_et1_tracker_robot_core
  PRIVATE
    ${AGENTIC_ET1_YAML_TARGET}
)

add_library(agentic_et1_tracker_policy_step_runner
  src/policy_step_runner.cpp
)

target_compile_features(agentic_et1_tracker_policy_step_runner PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_policy_step_runner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_policy_step_runner
  PUBLIC
    agentic_et1_tracker_policy_math
    agentic_et1_tracker_robot_core
    agentic_et1_tracker_trk
)

target_link_libraries(agentic_et1_tracker_runtime
  PUBLIC
    agentic_et1_tracker_control
    agentic_et1_tracker_policy_step_runner
    agentic_et1_tracker_velocity_policy
)

add_library(agentic_et1_tracker_policy
  src/deploy_config.cpp
)

target_compile_features(agentic_et1_tracker_policy PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_policy
  PRIVATE
    ${AGENTIC_ET1_YAML_TARGET}
)

add_library(agentic_et1_tracker_httplib INTERFACE)
target_include_directories(agentic_et1_tracker_httplib
  SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp_httplib
)

add_library(agentic_et1_tracker_http
  src/http_server.cpp
)

target_compile_features(agentic_et1_tracker_http PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_http
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_http
  PUBLIC
    agentic_et1_tracker_api
    agentic_et1_tracker_reference
  PRIVATE
    agentic_et1_tracker_httplib
    Threads::Threads
)

add_library(agentic_et1_tracker_app
  src/app_policy_path_guard.cpp
  src/app_config.cpp
  src/app_runner.cpp
  src/process_lock.cpp
)

target_compile_features(agentic_et1_tracker_app PUBLIC cxx_std_17)
target_include_directories(agentic_et1_tracker_app
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(agentic_et1_tracker_app
  PUBLIC
    agentic_et1_tracker_http
    agentic_et1_tracker_runtime
  PRIVATE
    agentic_et1_tracker_loco_upper
    agentic_et1_tracker_policy
    Threads::Threads
    ${AGENTIC_ET1_YAML_TARGET}
)

add_executable(agentic-et1-tracker
  src/main.cpp
)

target_compile_features(agentic-et1-tracker PRIVATE cxx_std_17)
target_link_libraries(agentic-et1-tracker
  PRIVATE
    agentic_et1_tracker_app
)

if(AGENTIC_ET1_BUILD_ONNX)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" AGENTIC_ET1_TARGET_PROCESSOR)
  set(AGENTIC_ET1_THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty")

  if(NOT ONNXRUNTIME_ROOT)
    if(AGENTIC_ET1_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
      set(ONNXRUNTIME_ROOT
          "${AGENTIC_ET1_THIRDPARTY_DIR}/onnxruntime-linux-x64-1.22.0"
          CACHE PATH "ONNX Runtime root")
    elseif(AGENTIC_ET1_TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
      set(ONNXRUNTIME_ROOT
          "${AGENTIC_ET1_THIRDPARTY_DIR}/onnxruntime-linux-aarch64-1.26.0"
          CACHE PATH "ONNX Runtime root")
    else()
      message(FATAL_ERROR
          "Unsupported ONNX Runtime processor: ${CMAKE_SYSTEM_PROCESSOR}. "
          "Set ONNXRUNTIME_ROOT to an ONNX Runtime directory.")
    endif()
  endif()

  find_path(AGENTIC_ET1_ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS "${ONNXRUNTIME_ROOT}/include"
    NO_DEFAULT_PATH
    REQUIRED
  )
  find_library(AGENTIC_ET1_ONNXRUNTIME_LIBRARY
    NAMES onnxruntime libonnxruntime.so.1.26.0 libonnxruntime.so.1.22.0
    PATHS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
    NO_DEFAULT_PATH
    REQUIRED
  )

  add_library(agentic_et1_tracker_onnxruntime SHARED IMPORTED)
  set_target_properties(agentic_et1_tracker_onnxruntime PROPERTIES
    IMPORTED_LOCATION "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${AGENTIC_ET1_ONNXRUNTIME_INCLUDE_DIR}"
  )

  get_filename_component(AGENTIC_ET1_ONNXRUNTIME_LIBRARY_DIR
    "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY}" DIRECTORY)

  add_library(agentic_et1_tracker_policy_onnx
    src/onnx_policy_runtime.cpp
  )
  target_compile_features(agentic_et1_tracker_policy_onnx PUBLIC cxx_std_17)
  target_include_directories(agentic_et1_tracker_policy_onnx
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(agentic_et1_tracker_policy_onnx
    PUBLIC
      agentic_et1_tracker_policy_step_runner
      agentic_et1_tracker_velocity_policy
    PRIVATE
      agentic_et1_tracker_policy_io_contract
      agentic_et1_tracker_onnxruntime
  )
  set_target_properties(agentic_et1_tracker_policy_onnx PROPERTIES
    BUILD_RPATH "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY_DIR}"
  )
endif()

if(AGENTIC_ET1_BUILD_TESTS OR AGENTIC_ET1_BUILD_PERF_SMOKE)
  enable_testing()
  find_package(Catch2 3 REQUIRED CONFIG)
endif()

if(AGENTIC_ET1_BUILD_TESTS)

  add_executable(agentic_et1_tracker_core_tests
    tests/controller_tests.cpp
    tests/core_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_core_tests
    PRIVATE
      agentic_et1_tracker_core
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_core_tests COMMAND agentic_et1_tracker_core_tests)

  add_executable(agentic_et1_tracker_robot_core_tests
    tests/robot_io_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_robot_core_tests
    PRIVATE
      agentic_et1_tracker_robot_core
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_robot_core_tests COMMAND agentic_et1_tracker_robot_core_tests)

  add_executable(agentic_et1_tracker_trk_tests
    tests/trk_loader_tests.cpp
    tests/trk_reference_alignment_tests.cpp
    tests/trk_synthetic_transition_tests.cpp
    tests/trk_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_trk_tests
    PRIVATE
      agentic_et1_tracker_trk
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_trk_tests COMMAND agentic_et1_tracker_trk_tests)

  add_executable(agentic_et1_tracker_loco_upper_tests
    tests/loco_lower_policy_tests.cpp
    tests/loco_upper_lowcmd_composer_tests.cpp
    tests/loco_upper_plan_compiler_tests.cpp
    tests/loco_upper_planner_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_loco_upper_tests
    PRIVATE
      agentic_et1_tracker_loco_upper
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_loco_upper_tests
           COMMAND agentic_et1_tracker_loco_upper_tests)

  add_executable(agentic_et1_tracker_api_tests
    tests/api_tests.cpp
    tests/reference_frame_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_api_tests
    PRIVATE
      agentic_et1_tracker_api
      agentic_et1_tracker_reference
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_api_tests COMMAND agentic_et1_tracker_api_tests)

  add_executable(agentic_et1_tracker_runtime_tests
    tests/runtime_bridge_tests.cpp
    tests/runtime_control_loop_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_runtime_tests
    PRIVATE
      agentic_et1_tracker_runtime
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_runtime_tests COMMAND agentic_et1_tracker_runtime_tests)

  add_executable(agentic_et1_tracker_policy_step_runner_tests
    tests/policy_step_runner_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_policy_step_runner_tests
    PRIVATE
      agentic_et1_tracker_policy_step_runner
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_policy_step_runner_tests
           COMMAND agentic_et1_tracker_policy_step_runner_tests)

  add_executable(agentic_et1_tracker_policy_tests
    tests/deploy_config_tests.cpp
    tests/observation_builder_tests.cpp
    tests/policy_io_contract_tests.cpp
    tests/policy_math_tests.cpp
    tests/velocity_policy_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_policy_tests
    PRIVATE
      agentic_et1_tracker_policy
      agentic_et1_tracker_policy_io_contract
      agentic_et1_tracker_policy_math
      agentic_et1_tracker_velocity_policy
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_policy_tests COMMAND agentic_et1_tracker_policy_tests)

  add_executable(agentic_et1_tracker_control_tests
    tests/fixstand_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_control_tests
    PRIVATE
      agentic_et1_tracker_control
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_control_tests
           COMMAND agentic_et1_tracker_control_tests)

  add_executable(agentic_et1_tracker_http_tests
    tests/http_server_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_http_tests
    PRIVATE
      agentic_et1_tracker_http
      agentic_et1_tracker_httplib
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_http_tests COMMAND agentic_et1_tracker_http_tests)

  add_executable(agentic_et1_tracker_app_tests
    tests/app_config_tests.cpp
    tests/app_runtime_factory_tests.cpp
    tests/app_runner_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_app_tests
    PRIVATE
      agentic_et1_tracker_app
      agentic_et1_tracker_httplib
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_app_tests COMMAND agentic_et1_tracker_app_tests)

  if(AGENTIC_ET1_BUILD_ONNX)
    add_executable(agentic_et1_tracker_policy_onnx_tests
      tests/onnx_policy_runtime_tests.cpp
    )
    target_link_libraries(agentic_et1_tracker_policy_onnx_tests
      PRIVATE
        agentic_et1_tracker_policy_onnx
        agentic_et1_tracker_policy
        Catch2::Catch2WithMain
    )
    set_target_properties(agentic_et1_tracker_policy_onnx_tests PROPERTIES
      BUILD_RPATH "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY_DIR}"
    )

    add_test(NAME agentic_et1_tracker_policy_onnx_tests
             COMMAND agentic_et1_tracker_policy_onnx_tests)
  endif()
endif()

if(AGENTIC_ET1_BUILD_PERF_SMOKE)
  add_executable(agentic_et1_tracker_perf_smoke_tests
    tests/perf_smoke_tests.cpp
  )
  target_link_libraries(agentic_et1_tracker_perf_smoke_tests
    PRIVATE
      agentic_et1_tracker_app
      agentic_et1_tracker_httplib
      Catch2::Catch2WithMain
  )

  add_test(NAME agentic_et1_tracker_perf_smoke_tests
           COMMAND agentic_et1_tracker_perf_smoke_tests)
  set_tests_properties(agentic_et1_tracker_perf_smoke_tests
    PROPERTIES
      LABELS perf
  )
endif()

if(AGENTIC_ET1_BUILD_ROBOT)
  set(AGENTIC_ET1_UNITREE_SDK2_ROOT "" CACHE PATH "Unitree SDK2 install root")
  set(AGENTIC_ET1_UNITREE_SDK2_PREFIXES)
  if(AGENTIC_ET1_UNITREE_SDK2_ROOT)
    list(APPEND AGENTIC_ET1_UNITREE_SDK2_PREFIXES "${AGENTIC_ET1_UNITREE_SDK2_ROOT}")
  endif()
  if(DEFINED ENV{UNITREE_SDK2_PATH})
    list(APPEND AGENTIC_ET1_UNITREE_SDK2_PREFIXES "$ENV{UNITREE_SDK2_PATH}")
  endif()
  list(APPEND AGENTIC_ET1_UNITREE_SDK2_PREFIXES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/unitree_sdk2_install"
    "/opt/unitree_robotics"
  )
  list(APPEND CMAKE_PREFIX_PATH ${AGENTIC_ET1_UNITREE_SDK2_PREFIXES})

  find_package(unitree_sdk2 REQUIRED CONFIG)

  add_library(agentic_et1_tracker_unitree_sdk_robot
    src/unitree_sdk_robot_io.cpp
  )
  target_compile_features(agentic_et1_tracker_unitree_sdk_robot PUBLIC cxx_std_17)
  target_include_directories(agentic_et1_tracker_unitree_sdk_robot
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(agentic_et1_tracker_unitree_sdk_robot
    PUBLIC
      agentic_et1_tracker_robot_core
      unitree_sdk2
      Threads::Threads
  )

  if(AGENTIC_ET1_BUILD_TESTS)
    add_executable(agentic_et1_tracker_unitree_sdk_robot_tests
      tests/unitree_sdk_robot_io_tests.cpp
    )
    target_link_libraries(agentic_et1_tracker_unitree_sdk_robot_tests
      PRIVATE
        agentic_et1_tracker_unitree_sdk_robot
        Catch2::Catch2WithMain
    )

    add_test(NAME agentic_et1_tracker_unitree_sdk_robot_tests
             COMMAND agentic_et1_tracker_unitree_sdk_robot_tests)
  endif()
endif()

if(AGENTIC_ET1_BUILD_ONNX AND AGENTIC_ET1_BUILD_ROBOT)
  target_sources(agentic_et1_tracker_app
    PRIVATE
      src/app_runtime_factory_real.cpp
  )
  target_compile_definitions(agentic_et1_tracker_app
    PUBLIC
      AGENTIC_ET1_TRACKER_REAL_FACTORY=1
  )
  target_link_libraries(agentic_et1_tracker_app
    PRIVATE
      agentic_et1_tracker_policy
      agentic_et1_tracker_policy_onnx
      agentic_et1_tracker_unitree_sdk_robot
  )
  set_target_properties(agentic-et1-tracker PROPERTIES
    BUILD_RPATH "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY_DIR}"
  )
  if(AGENTIC_ET1_BUILD_TESTS)
    set_target_properties(agentic_et1_tracker_app_tests PROPERTIES
      BUILD_RPATH "${AGENTIC_ET1_ONNXRUNTIME_LIBRARY_DIR}"
    )
  endif()
else()
  target_sources(agentic_et1_tracker_app
    PRIVATE
      src/app_runtime_factory_stub.cpp
  )
  target_compile_definitions(agentic_et1_tracker_app
    PUBLIC
      AGENTIC_ET1_TRACKER_REAL_FACTORY=0
  )
endif()
