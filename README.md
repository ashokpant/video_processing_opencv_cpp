# OpenCV C++ Examples

##  Dependencies
Install OpenCV 2.4.x [Script](https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh)
Google Flags 2.1.0 ($ sudo apt-get install libgflags-dev)

## Algorithms
* Card detection
* Image quality detection
* Line segment detection

## Examples:
* Card detection in videos
```
examples/carddetector_video.cpp
```
* Image quality detection in videos
```
examples/imgquality_video.cpp
```
## Build
```bash
mkdir build
cd build
cmake ../
make
```       

## References
#### Straight line segment extractor ([linedetector](//https://github.com/ashokpant/linedetector))

* Lee, Jin Han, et al. "Place recognition using straight lines for vision-based
SLAM." Robotics and Automation (ICRA), 2013 IEEE International Conference on.
IEEE, 2013.

* Lee, Jin Han, et al. "Outdoor place recognition in urban environments using
straight lines." Robotics and Automation (ICRA), 2014 IEEE International
Conference on. IEEE, 2014.

* Zhang, Guoxuan, et al. "Building a 3-D Line-Based Map Using Stereo SLAM."
Robotics, IEEE Transactions on 31.6 (2015): 1364-1377.
