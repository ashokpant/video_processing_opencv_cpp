// Created by: Ashok Kumar Pant
// Created on: 7/7/20


#ifndef VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX
#define VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX

#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class ImgQuality {

    static double getBrightnessScore(const Mat &img);

    static double getBlurScore(const Mat &img);

    static void hasGlare(const Mat &img, double &min, double &max);

    static void hasGlare1(const Mat &img, double &min, double &max);
};

#endif //VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX
