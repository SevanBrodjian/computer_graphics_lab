cmake_minimum_required(VERSION 3.24)
project(hw4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(PNG REQUIRED)

add_library(hw4_common
    shader_program.cpp
    scene_loader.cpp
    png_loader.cpp)

target_include_directories(hw4_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen
    ${PNG_INCLUDE_DIRS})

target_link_libraries(hw4_common PUBLIC ${PNG_LIBRARIES})

add_executable(opengl_renderer
    opengl_renderer.cpp)

target_link_libraries(opengl_renderer PRIVATE hw4_common GLUT::GLUT GLEW::GLEW OpenGL::GL)

target_include_directories(opengl_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

add_executable(normal_map_renderer
    normal_map_renderer.cpp)

target_link_libraries(normal_map_renderer PRIVATE hw4_common GLUT::GLUT GLEW::GLEW OpenGL::GL)

target_include_directories(normal_map_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)
