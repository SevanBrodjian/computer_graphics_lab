cmake_minimum_required(VERSION 3.24)
project(hw4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)# C++ 14 for Eigen compatibility, required
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(PNG REQUIRED)

add_executable(opengl_renderer
    opengl_renderer.cpp
    scene_loader.cpp
    texture_loader.cpp)

target_include_directories(opengl_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(opengl_renderer PRIVATE SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")

target_link_libraries(opengl_renderer PRIVATE GLUT::GLUT GLEW::GLEW OpenGL::GL PNG::PNG)

# Silence deprecation warnings (annoying, mostly from Eigen)
if(APPLE)
  target_compile_definitions(opengl_renderer PRIVATE GL_SILENCE_DEPRECATION)
  target_compile_options(opengl_renderer PRIVATE -Wno-deprecated-declarations)
endif()