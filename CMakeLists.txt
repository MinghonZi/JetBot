cmake_minimum_required(VERSION 3.15...3.26)
project(JetBot)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # ensure standard is supported
set(CMAKE_CXX_EXTENSIONS OFF)  # keep compiler extensions off

# Initialise Git submodules
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS ${PROJECT_SOURCE_DIR}/.git)
    message(STATUS "Initialise submodules")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}")
    endif()
endif()

# Ensure Python and pybind11 existence
find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_PATH ${CMAKE_SOURCE_DIR}/extern/pybind11)
if(NOT EXISTS ${PYBIND11_PATH}/CMakeLists.txt)
    message(FATAL_ERROR "${PYBIND11_PATH} were not initialised")
endif()
add_subdirectory(${PYBIND11_PATH})

include_directories(include)

add_subdirectory(pywrapcc)
add_subdirectory(tests)
