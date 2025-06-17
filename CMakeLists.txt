cmake_minimum_required(VERSION 3.10)

# vcpkg toolchain 연동
set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

project(PMXViewer)

# C++20 명시
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GLFW / GLAD 찾기
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# 실행 파일
add_executable(PMXViewer main.cpp Pmx.cpp)

# 라이브러리 연결
target_link_libraries(PMXViewer PRIVATE glfw glad::glad glm::glm)
