cmake_minimum_required(VERSION 3.16)
project(video_processing_opencv_cpp)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 14)

add_executable(idcard_detector src/idcard_detector.cpp)
target_link_libraries(idcard_detector ${OpenCV_LIBS})