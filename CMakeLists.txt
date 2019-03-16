cmake_minimum_required(VERSION 3.7)
project(snow)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# RenderBox visualization only supports some platforms

if (APPLE)
    set(USE_RENDERBOX YES)
endif ()

# Dependencies

if (USE_RENDERBOX)
    add_subdirectory(vendor/renderbox)
endif ()

# Static library

file(GLOB_RECURSE LIB_SOURCE_FILES lib/*.cpp)

add_library(snowlib ${LIB_SOURCE_FILES})
target_include_directories(snowlib
        PUBLIC vendor/eigen/Eigen
        PUBLIC vendor/renderbox/include
        PUBLIC vendor/renderbox/vendor/glm)

# App

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

add_executable(snow ${SOURCE_FILES})
target_link_libraries(snow snowlib)

if (USE_RENDERBOX)
    add_compile_definitions(USE_RENDERBOX)
    target_link_libraries(snow renderbox)
endif ()

# Tests

find_package(Boost REQUIRED COMPONENTS unit_test_framework)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
endif ()

file(GLOB_RECURSE TEST_SOURCE_FILES tests/*.cpp)

add_executable(snow_tests ${TEST_SOURCE_FILES})
if (Boost_FOUND)
    target_link_libraries(snow_tests snowlib ${Boost_LIBRARIES})
endif ()
