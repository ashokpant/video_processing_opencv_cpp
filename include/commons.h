// Created by: Ashok Kumar Pant
// Created on: 7/1/20


#ifndef VIDEO_PROCESSING_OPENCV_CPP_COMMONS_H
#define VIDEO_PROCESSING_OPENCV_CPP_COMMONS_H

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>

using namespace cv;

Mat resize(const Mat &img, int width, int height, bool resizeLarger);

Mat resizeLargestDim(const Mat &img, int size, bool resizeLarger);

double euclideanDist(double x1, double y1, double x2, double y2);

#endif //VIDEO_PROCESSING_OPENCV_CPP_COMMONS_H
