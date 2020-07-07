cmake_minimum_required(VERSION 3.16)
project(video_processing_opencv_cpp)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 14)

add_executable(lsd src/lsd_main.cpp src/lsdlines.cpp src/commons.cpp)
target_link_libraries(lsd ${OpenCV_LIBS})

add_executable(linedetection_image src/linedetection_image.cpp src/linedetector.cpp)
target_link_libraries(linedetection_image ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})
add_executable(linedetection_video src/linedetection_video.cpp src/linedetector.cpp)
target_link_libraries(linedetection_video ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_executable(idcard_detector src/carddetector_video.cpp src/carddetector.cpp src/linedetector.cpp)
target_link_libraries(idcard_detector ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_library(imgquality src/imgquality.cpp)
target_link_libraries(imgquality ${OpenCV_LIBS})