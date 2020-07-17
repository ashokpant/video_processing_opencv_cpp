cmake_minimum_required(VERSION 3.16)
project(video_processing_opencv_cpp)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 20)

add_executable(lsd examples/lsd_main.cpp src/lsdlines.cpp src/commons.cpp)
target_link_libraries(lsd ${OpenCV_LIBS})

add_executable(linedetection_image examples/linedetection_image.cpp src/linedetector.cpp)
target_link_libraries(linedetection_image ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_executable(linedetection_video examples/linedetection_video.cpp src/linedetector.cpp)
target_link_libraries(linedetection_video ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_executable(idcard_detector examples/carddetector_video.cpp src/carddetector.cpp src/linedetector.cpp)
target_link_libraries(idcard_detector ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_executable(facedetection_video examples/facedetector_video.cpp src/facedetector.cpp src/commons.cpp)
target_link_libraries(facedetection_video ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_executable(imgquality_video examples/imgquality_video.cpp src/imgquality.cpp)
target_link_libraries(imgquality_video ${GOOGLE_LIBRARIES} ${OpenCV_LIBS})

add_library(imgquality src/imgquality.cpp)
target_link_libraries(imgquality ${OpenCV_LIBS})

add_library(idcard SHARED src/imgquality.cpp src/carddetector.cpp src/linedetector.cpp src/facedetector.cpp)
target_link_libraries(idcard ${OpenCV_LIBS})

