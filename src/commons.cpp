// Created by: Ashok Kumar Pant
// Created on: 7/1/20

#include "commons.h"

Mat resize(const Mat &img, int width, int height, bool resizeLarger) {
    int h = img.size().height;
    int w = img.size().width;
    Mat resized = Mat();
    if (width > 0 && height > 0) {
        if (!(resizeLarger && h <= height && w <= width)) {
            cv::resize(img, resized, Size(width, height));
        } else {
            resized = img;
        }
    } else if (width > 0) {
        if (!(resizeLarger && w <= width)) {
            double wPercent = (width / (w * 1.0));
            int hSize = (int) (h * wPercent);
            cv::resize(img, resized, Size(width, hSize));
        } else {
            resized = img;
        }
    } else if (height > 0 && !(resizeLarger && h <= height)) {
        double hPercent = (height / (h * 1.0));
        int wSize = (int) (w * hPercent);
        cv::resize(img, resized, Size(wSize, height));
    } else {
        resized = img;
    }
    return resized;
}

Mat resizeLargestDim(const Mat &img, int size, bool resizeLarger) {
    int h = img.rows;
    int w = img.cols;
    if (h > w) {
        return resize(img, 0, size, resizeLarger);
    } else {
        return resize(img, size, 0, resizeLarger);
    }
}

double euclideanDist(double x1, double y1, double x2, double y2) {
    double x = x1 - x2;
    double y = y1 - y2;
    double dist;
    dist = pow(x, 2) + pow(y, 2);
    dist = sqrt(dist);
    return dist;
}