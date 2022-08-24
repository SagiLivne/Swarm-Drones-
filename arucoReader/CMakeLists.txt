cmake_minimum_required(VERSION 3.22)
project(2022summerCourse)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_package(OpenCV 3.0 QUIET)
include_directories(include)
add_library(aruco src/aruco.cpp)
add_library(drone src/drone.cpp)
target_link_libraries(aruco -lpthread ${OpenCV_LIBS})
add_executable(runAruco
        runAruco.cpp)
target_link_libraries(runAruco aruco drone -lctello -std=c++11 -pthread nlohmann_json::nlohmann_json )
