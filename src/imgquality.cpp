// Created by: Ashok Kumar Pant
// Created on: 7/7/20


#include "../include/imgquality.h"

double ImgQuality::getBrightnessScore(const Mat &img) {
    // Output range = [0-1], higher the better
    try {
        Mat hsv = Mat();
        cv::cvtColor(img, hsv, COLOR_BGR2HSV);
        Scalar mean = cv::mean(hsv);
        return mean.val[2] / 255.0;
    } catch (Exception &e) {
        cerr << e.what() << endl;
        return 0;
    }
}

double ImgQuality::getBlurScore(const Mat &img) {
    // Focus measure: Output range: >0, >100 is better
    try {
        Mat gray;
        if (img.channels() > 2) {
            cv::cvtColor(img, gray, COLOR_BGR2GRAY);
        } else {
            gray = img.clone();
        }
        Mat dst = Mat();
        cv::Laplacian(gray, dst, CV_64F);
        Mat mu = Mat();
        Mat sigma = Mat();
        cv::meanStdDev(dst, mu, sigma);
        double fm = sigma.at<double>(0, 0) * sigma.at<double>(0, 0);
        return fm;
    } catch (Exception &e) {
        cerr << e.what() << endl;
        return 0;
    }
}

void ImgQuality::getGlareScore(const Mat &img, double &min, double &max) {
    //Glare if max >= 253.0 && min <= 0.0
    try {
        Mat gray;
        if (img.channels() > 2) {
            cv::cvtColor(img, gray, COLOR_BGR2GRAY);
        } else {
            gray = img.clone();
        }
        cv::GaussianBlur(gray, gray, Size(7, 7), 1.666);
        cv::minMaxLoc(gray, &min, &max);
    } catch (Exception &e) {
        cerr << e.what() << endl;
    }
}

void ImgQuality::getGlareScore1(const Mat &img, double &min, double &max) {
    try {
        Mat hsv = Mat();
        cv::cvtColor(img, hsv, COLOR_BGR2HSV);
        vector<Mat> split;
        cv::split(hsv, split);
        Mat gray = split[2];
        cv::minMaxLoc(gray, &min, &max);

    } catch (Exception &e) {
        cerr << e.what() << endl;
    }
}
