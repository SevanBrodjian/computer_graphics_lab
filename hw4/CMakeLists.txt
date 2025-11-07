cmake_minimum_required(VERSION 3.24)
project(hw4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(PNG REQUIRED)

add_executable(opengl_renderer
    opengl_renderer.cpp
    scene_loader.cpp
    texture_loader.cpp)

target_include_directories(opengl_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

target_link_libraries(opengl_renderer PRIVATE GLUT::GLUT GLEW::GLEW OpenGL::GL PNG::PNG)
