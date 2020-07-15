// Created by: Ashok Kumar Pant
// Created on: 7/7/20


#ifndef VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX
#define VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class ImgQuality {
public:
    static double getBrightnessScore(const Mat &img);

    static double getBlurScore(const Mat &img);

    static void getGlareScore(const Mat &img, double &min, double &max);

    static void getGlareScore1(const Mat &img, double &min, double &max);

    static double getGlareScore2(const Mat &img);
};

#endif //VIDEO_PROCESSING_OPENCV_CPP_IMGQUALITY_HXX
