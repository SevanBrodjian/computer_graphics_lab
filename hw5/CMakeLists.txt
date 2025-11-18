cmake_minimum_required(VERSION 3.24)
project(hw5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14) # C++ 14 for Eigen compatibility, required
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

add_executable(smooth
    smooth.cpp
    scene_loader.cpp)

target_include_directories(smooth PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(smooth PRIVATE GLUT::GLUT GLEW::GLEW OpenGL::GL)
