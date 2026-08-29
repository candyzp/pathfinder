cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

project(pathfinder VERSION 1.0.0)

add_library(${PROJECT_NAME} SHARED
    src/main.cpp
    src/creator.cpp
    src/pathfinder_single.cpp
    src/solver_dashboard.cpp
    src/flappy.cpp
    src/checker.cpp
    src/debug.cpp
    # Add any extra C++ source files here
)

# Clean single-controller Pathfinder. The V7-V13 helper/team experiments are
# intentionally outside the active build. pathfinder_single.cpp wraps the plain
# all-game-mode Pathfinder with bounded no-progress retries and Stop preservation.

if (NOT DEFINED ENV{GEODE_SDK})
    message(FATAL_ERROR "Please define GEODE_SDK environment variable to point to Geode")
else()
    message(STATUS "Found Geode: $ENV{GEODE_SDK}")
endif()

add_subdirectory($ENV{GEODE_SDK} ${CMAKE_CURRENT_BINARY_DIR}/geode)
add_subdirectory(gd-sim)

CPMAddPackage("gh:camila314/UIBuilder#main")
CPMAddPackage("gh:maxnut/GDReplayFormat#gdr2")

# The old strip target is for desktop macOS. Geode's iOS pipeline produces an
# arm64 iPhoneOS dylib and should be left untouched for packaging/signing.
if (APPLE AND NOT GEODE_TARGET_PLATFORM STREQUAL "iOS")
add_custom_target(strip ALL
    COMMAND $<$<CONFIG:Debug>:echo> strip -x $<TARGET_FILE:${PROJECT_NAME}>
    DEPENDS ${PROJECT_NAME}
)
endif()

setup_geode_mod(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-DDEBUG_MODE=1>)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-g> -O3 -fno-math-errno -fno-trapping-math -fvisibility=hidden -fomit-frame-pointer)
target_link_libraries(${PROJECT_NAME} gd-sim UIBuilder libGDR)
