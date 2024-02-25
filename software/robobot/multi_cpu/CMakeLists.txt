cmake_minimum_required(VERSION 3.0)

project(aaa)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${dlib_INCLUDE_DIR})

add_executable(aaa main.cpp utime.cpp)
target_link_libraries(aaa ${OpenCV_LIBS})

install(TARGETS aaa RUNTIME DESTINATION bin)
