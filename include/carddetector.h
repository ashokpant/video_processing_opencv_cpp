// Created by: Ashok Kumar Pant
// Created on: 6/26/20
#ifndef _IDDETECTOR_H_
#define _IDDETECTOR_H_

#include "linedetector.h"
#include <chrono>
#include <bits/stdc++.h>

using namespace std::chrono;


class IdCardResult {
private:
    Rect rect;
    vector<Point> polygon;
    Mat img;

public:
    const Rect &getRect() const {
        return rect;
    }

    void setRect(const Rect &rect) {
        IdCardResult::rect = rect;
    }

    const vector<Point> &getPolygon() const {
        return polygon;
    }

    void setPolygon(const vector<Point> &polygon) {
        IdCardResult::polygon = polygon;
    }

    const Mat &getImg() const {
        return img;
    }

    void setImg(const Mat &img) {
        IdCardResult::img = img;
    }

    friend ostream &operator<<(ostream &os, const IdCardResult &result) {
        os << "rect: " << result.rect << " polygon: " << result.polygon << " img: " << result.img;
        return os;
    }
};

class IdCardDetector {

    int detectV1(Mat &img, IdCardResult &result);

    // camPortrait: camera orientation
    // docPortrait: document layout orientation
    int detectV3(Mat &img, IdCardResult &result, bool camPortrait = false, bool docPortrait = false);

    LineDetector lineDetector;

public:
    int detect(Mat &img, IdCardResult &result, bool camPortrait = true, bool docPortrait = false) {
        return detectV3(img, result, camPortrait, docPortrait);
    }

    IdCardDetector();

    ~ IdCardDetector() = default;
};


#endif