cmake_minimum_required(VERSION 3.24)
project(hw3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

add_executable(opengl_renderer
    opengl_renderer.cpp
    scene_loader.cpp)

target_include_directories(opengl_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(opengl_renderer PRIVATE GLUT::GLUT GLEW::GLEW OpenGL::GL)
