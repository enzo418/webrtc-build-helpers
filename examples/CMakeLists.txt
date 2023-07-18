cmake_minimum_required(VERSION 3.14)
project(webrtc_demo VERSION 1.0.0)

# ----------------------- OPTIONS ---------------------- #
include(CMakeDependentOption)

set(WEBRTC_BUILD_TYPE "Release" CACHE STRING "Build WebRTC type: Debug | Release")

# disabled by default, compilers change warnings over time
option(WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# see the webrtc cmake script for more details
option(WEBRTC_INCLUDE_DEFAULT_AUDIO "Use WebRTC default audio implementation" ON)

# -------------- COMPILATION CONFIGURATION ------------- #
set (CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # clangd

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# ------------------ MAIN CMAKE FILES ------------------ #

# Dependecies file
include(${CMAKE_CURRENT_LIST_DIR}/Dependency.cmake)

# ----------------------- EXAMPLES ----------------------- #
set(ALL_INCLUDE_DIRS 
  ${DEPENDENCY_INCLUDE_DIR} 
  ${DEPENDENCY_INCLUDE_LIST} 
  ${DEPENDENCY_INCLUDE_DIR}/webrtc  # allows to include absl/... instead of webrtc/absl/...
                                    # like all the webRTC code does
)

file(GLOB_RECURSE examples "src/*.cpp")

foreach(filename ${examples})
    string(REGEX REPLACE
          "([/\\][A-Za-z0-9_]+)+[/\\]([A-Za-z0-9_]+)\.c(pp)?"
          "\\2" example_name ${filename})

    add_executable(${example_name} 
        ${filename} 
    )
    
    # ------------- INCLUDES / LIBS DIRECTORIES ------------ #
    target_include_directories(${example_name} SYSTEM PUBLIC ${ALL_INCLUDE_DIRS})

    target_link_directories(${example_name} PUBLIC ${DEPENDENCY_LIB_DIR})

    # ---------------- DEPENDECIES / LIBRARY --------------- #
    add_dependencies(${example_name} ${DEPENDENCY_LIST})

    target_link_libraries(${example_name} ${DEPENDENCY_LIBS})

    target_compile_options(${example_name} PRIVATE -Wall -Wextra -Wpedantic)

    target_compile_definitions(${example_name} PUBLIC ${OBSERVER_DEPENDENCY_COMPILATION_OPTIONS})

    if (${WEBRTC_INCLUDE_DEFAULT_AUDIO})
      target_compile_definitions(
        ${example_name} PUBLIC 
            WEBRTC_INCLUDE_DEFAULT_AUDIO
      )
    endif()
endforeach()


# ------------------------ TESTS ----------------------- #
