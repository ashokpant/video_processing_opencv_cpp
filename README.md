# OpenCV C++ Examples

##  Dependencies
Install OpenCV [Script](https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh)


## Algorithms
* Edge detection


### Straight line segment extractor ([linedetector](//https://github.com/ashokpant/linedetector))

Simple but efficient/effective line segement detector.
This detector, used in works listed below, extracts line segments from images more effectively than classical Hough transform or LSD.
This detector also has a function that merges noisy-broken short line segments into one segment for more reliable detection.

Please cite one of these papers if you use this code in your research:

Lee, Jin Han, et al. "Place recognition using straight lines for vision-based
SLAM." Robotics and Automation (ICRA), 2013 IEEE International Conference on.
IEEE, 2013.

Lee, Jin Han, et al. "Outdoor place recognition in urban environments using
straight lines." Robotics and Automation (ICRA), 2014 IEEE International
Conference on. IEEE, 2014.

Zhang, Guoxuan, et al. "Building a 3-D Line-Based Map Using Stereo SLAM."
Robotics, IEEE Transactions on 31.6 (2015): 1364-1377.

# Dependency
Opencv 2.4.x and upper versions
Google Flags 2.1.0 ($ sudo apt-get install libgflags-dev)

# Usage

$ mkdir build
$ cd build
$ cmake ../
$ make
$ ./linedetection_image -i ../img/squre.jpg
$ ./linedetection_video

