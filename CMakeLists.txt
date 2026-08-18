cmake_minimum_required(VERSION 3.23)
project(GLTest)

find_package(GLEW CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm CONFIG REQUIRED)

add_executable(GLTest gltesting.cpp)
target_link_libraries(GLTest PRIVATE GLEW::GLEW glfw glm::glm)