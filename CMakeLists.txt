cmake_minimum_required(VERSION 3.15...3.26)
project(JetBot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # ensure standard is supported
set(CMAKE_CXX_EXTENSIONS OFF)  # keep compiler extensions off

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

find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_PATH ${CMAKE_SOURCE_DIR}/extern/pybind11)
if(NOT EXISTS ${PYBIND11_PATH}/CMakeLists.txt)
    message(FATAL_ERROR "${PYBIND11_PATH} were not initialised")
endif()
add_subdirectory(${PYBIND11_PATH})

include_directories(include)

add_subdirectory(pywrapcc)
add_subdirectory(tests)
