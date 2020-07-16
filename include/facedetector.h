// Created by: Ashok Kumar Pant
// Created on: 7/16/20


#ifndef VIDEO_PROCESSING_OPENCV_CPP_FACEDETECTOR_H
#define VIDEO_PROCESSING_OPENCV_CPP_FACEDETECTOR_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;


class FaceResult {
private:
    vector<Rect> faces;
    vector<vector<Point>> landmarks;

public:
    const vector<Rect> &getFaces() const {
        return faces;
    }

    void setFaces(const vector<Rect> &_faces) {
        FaceResult::faces = _faces;
    }

    const vector<vector<Point>> &getLandmarks() const {
        return landmarks;
    }

    void setLandmarks(const vector<vector<Point>> &_landmarks) {
        FaceResult::landmarks = _landmarks;
    }

    std::string toString() {
        ostringstream ss;
        ss << this;
        return ss.str();
    }

};

class FaceDetector {
    int
    detectFacesV1(Mat &img, FaceResult &result, int minSize = 50, bool findLargest = false, bool findLandmarks = true);
    CascadeClassifier faceCascade;

public:
    int detect(Mat &img, FaceResult &result, int minSize = 50, bool findLargest = false, bool findLandmarks = true) {
        return detectFacesV1(img, result, minSize, findLargest, findLandmarks);
    }

    FaceDetector(const string& faceCascadeFilename);

    ~ FaceDetector() = default;
};

#endif //VIDEO_PROCESSING_OPENCV_CPP_FACEDETECTOR_H
