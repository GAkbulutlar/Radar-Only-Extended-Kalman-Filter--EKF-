cmake_minimum_required(VERSION 3.14)
project(radar_only_ekf_cpp17 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(radar_ekf
    src/main.cpp
    src/kalman_filter.cpp
)

target_include_directories(radar_ekf PRIVATE include)
target_link_libraries(radar_ekf PRIVATE Eigen3::Eigen)
