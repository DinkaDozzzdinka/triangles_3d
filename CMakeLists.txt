cmake_minimum_required(VERSION 3.16)
project(triangles_3d CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
add_compile_options(-Wall -Wextra -Wpedantic -Werror -O2)
endif()

add_executable(triangles_3d src/main.cpp)

target_compile_features(triangles_3d PUBLIC cxx_std_17)

target_include_directories(triangles_3d PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

