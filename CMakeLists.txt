cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

project(PMXViewer)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 CONFIG REQUIRED)
find_package(glad  CONFIG REQUIRED)
find_package(glm   CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED)

add_executable(PMXViewer
    "main.cpp" "stb_image.h" "stb_image_resize2.h"
    "EncodingHelper.h"
    "Pmx.h" "Pmx.cpp" "Vmd.h"
    "IKSolver.h" "IKSolver.cpp"
    "NodeManager.h" "NodeManager.cpp"
    "MorphManager.h" "MorphManager.cpp"
    "PMXActor.h" "PMXActor.cpp"
    "miniaudio.h"
    "PhysicsManager.h" "PhysicsManager.cpp"
 )

target_link_libraries(PMXViewer PRIVATE
    glfw
    glad::glad
    glm::glm
    BulletDynamics
    BulletCollision
    LinearMath
)

target_compile_definitions(PMXViewer PRIVATE NOMINMAX)
