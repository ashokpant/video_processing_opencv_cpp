#ifndef LINEDETECTOR_H_
#define LINEDETECTOR_H_

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
//https://github.com/ashokpant/linedetector
using namespace std;
using namespace cv;
struct SEGMENT {
    float x1, y1, x2, y2, angle;
    int label;
};

class LineDetector {
    float PI = 3.141592653589793238463;
public:
    LineDetector(float threshDist, int threshLength) {
        thresholdDist = threshDist;
        thresholdLength = threshLength;
        initLabel = 0;
    }

    LineDetector() {
        thresholdDist = 1.5;
        thresholdLength = 20;
        initLabel = 0;
    }

    ~LineDetector() = default;;

    template<class tType>
    void incidentPoint(tType *pt, Mat &l);

    void mergeLines(SEGMENT *Seg1, SEGMENT *Seg2, SEGMENT *SegMerged);

    bool getPointChain(const Mat &img, Point pt, Point *chained_pt, int
    &direction, int step);

    // bool getPointChain( const Mat & img, const Mat & scales, const Point pt, Point * chained_pt, int
    // & direction, int step );
    double distPointLine(const Mat &p, Mat &l);

    bool mergeSegments(SEGMENT *seg1, SEGMENT *seg2, SEGMENT *seg_merged);

    void extractSegments(vector<Point2i> *points, vector<SEGMENT> *segments);

    void lineDetection(Mat &src, vector<SEGMENT> &segments_all, bool merge = true);

    void pointInboardTest(Mat &src, Point2i *pt);

    void getAngle(SEGMENT *seg);

    void additionalOperationsOnSegments(Mat &src, SEGMENT *seg);

    void drawArrow(Mat &mat, const SEGMENT *seg, Scalar bgr = Scalar(0, 255, 0),
                   int thickness = 1, bool directed = true);

    void detect(Mat &src, vector<array<int, 5>> &lines, bool merge = true) {
        std::vector<SEGMENT> lsdLines = vector<SEGMENT>();
        lineDetection(src, lsdLines, merge);
        for (SEGMENT line:lsdLines) {
            std::array<int, 5> l = {};
            l[0] = line.x1;
            l[1] = line.y1;
            l[2] = line.x2;
            l[3] = line.y2;
            l[4] = line.angle * 180 / PI;
            lines.push_back(l);
        }
    }

private:
    int initLabel, imageWidth{}, imageHeight{}, thresholdLength;
    float thresholdDist;
};

#endif
