cmake_minimum_required(VERSION 3.16)
project(triangles_3d LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(triangles_3d src/main.cpp)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(triangles_3d PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()

add_library(triangle_input STATIC src/input.cpp)

target_compile_features(triangles_3d PUBLIC cxx_std_17)

target_include_directories(triangle_input PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(triangles_3d PRIVATE triangle_input)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_library(glad STATIC third_party/glad/src/gl.c)

target_include_directories(glad PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include"
)

add_executable(window_demo src/window_demo.cpp)
target_link_libraries(window_demo PRIVATE glad glfw OpenGL::GL)

add_library(triangle_geometry STATIC 
    src/geometry.cpp
    src/geometry2d.cpp)

target_include_directories(triangle_geometry PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(triangles_3d PRIVATE triangle_geometry)


include(CTest)
if (BUILD_TESTING)
    if (POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(input_tests tests/input_test.cpp)
    target_link_libraries(input_tests PRIVATE triangle_input GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(input_tests)

    
    add_executable(geometry_tests 
        tests/geometry_test.cpp
        tests/geometry2d_test.cpp)
    target_link_libraries(geometry_tests PRIVATE triangle_geometry GTest::gtest_main)
    gtest_discover_tests(geometry_tests)
    endif()
