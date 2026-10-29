cmake_minimum_required(VERSION 3.22)
project(bullet_render LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(FetchContent)
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/build/deps")
FetchContent_Declare(
  godot_cpp
  GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
  GIT_TAG e83fd0904c13356ed1d4c3d09f8bb9132bdc6b77
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/build/deps/godot-cpp")
set(GODOTCPP_BUILD_PROFILE
    "${CMAKE_SOURCE_DIR}/build_profile.json"
    CACHE FILEPATH "" FORCE)
set(GODOTCPP_DISABLE_EXCEPTIONS OFF CACHE BOOL "Native adapter catches errors" FORCE)
# Share Mesa/LLVM's runtime instead of exporting a conflicting static C++ runtime.
set(GODOTCPP_USE_STATIC_CPP OFF CACHE BOOL "Use the system shared C++ runtime" FORCE)
set(CMAKE_PROJECT_INCLUDE "${CMAKE_SOURCE_DIR}/cmake/godot_profile.cmake")
FetchContent_MakeAvailable(godot_cpp)
unset(CMAKE_PROJECT_INCLUDE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq>=4.3.5)
find_package(Threads REQUIRED)
add_library(zmq_camera SHARED src/native/zmq_camera_adapter.cpp src/native/register_types.cpp)
target_include_directories(zmq_camera PRIVATE "${CMAKE_SOURCE_DIR}/../bullet_msgs/include")
target_link_libraries(zmq_camera PRIVATE godot-cpp PkgConfig::ZMQ Threads::Threads)
target_compile_options(zmq_camera PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(zmq_camera PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/godot/bin")

find_program(GODOT_EXECUTABLE godot REQUIRED)
# shortcut: Godot headless import has shutdown race #111048; remove delay when
# the engine fixes it.
add_custom_command(
  TARGET zmq_camera
  POST_BUILD
  COMMAND "${GODOT_EXECUTABLE}" --headless --path "${CMAKE_SOURCE_DIR}/godot"
          --editor --import --frame-delay 1000 --quit
  COMMENT "Register native extension with Godot")
