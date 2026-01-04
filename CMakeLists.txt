cmake_minimum_required(VERSION 3.20.0)
project(RayKin VERSION 0.1.0 LANGUAGES C CXX)
# Packages
find_package(Eigen3 REQUIRED)
find_package(tinyxml2 REQUIRED)

#Install raylib lcoally
include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.0 # or the latest stable tag
)
FetchContent_MakeAvailable(raylib)

file(GLOB_RECURSE SOURCES
    src/main.cpp
    src/*/*.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/sim
    ${CMAKE_SOURCE_DIR}/src/viz
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)

# C++ 23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



# CTEST & CPACK
include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
