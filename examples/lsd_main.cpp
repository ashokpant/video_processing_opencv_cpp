// Created by: Ashok Kumar Pant
// Created on: 6/26/20

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include "../include/commons.h"
#include "../include/lsdlines.h"

using namespace std;
using namespace cv;


vector<line_float_t> lsd_detect_lines(const Mat &img) {
    int oh = img.size().height;
    int ow = img.size().width;
    Mat image = resizeLargestDim(img, 640, true);
    int h = image.size().height;
    int w = image.size().width;
    float scale = (float) ow / (float) w;
    cout << "Image size:(" << ow << "," << oh << ")  scaled: (" << w << "," << h << ")\n";
    Mat gray = Mat();
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    unsigned char *bytes = gray.data;
    boundingbox_t bbox = boundingbox_t();
    bbox.x = 0;
    bbox.y = 0;
    bbox.width = w;
    bbox.height = h;
    std::vector<line_float_t> lsdLines = vector<line_float_t>();
    int success = LsdLineDetector(bytes, w, h, 1.0, 1.0, bbox, lsdLines);
    vector <line_float_t> lines = vector<line_float_t>();
    if (success == 0) {
        for (line_float_t _line : lsdLines) {
            line_float_t line = line_float_t();
            line.startx = _line.startx * scale;
            line.starty = _line.starty * scale;
            line.endx = _line.endx * scale;
            line.endy = _line.endy * scale;
            lines.push_back(line);
        }
    }
    return lines;
}

int main(int argc, char **argv) {
//    VideoCapture capture = cv::VideoCapture(0);
    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", 1);
    while (true) {
        Mat img;
        bool s = capture.read(img);
        if (!s) {
            break;
        }
        vector<line_float_t> lines = lsd_detect_lines(img);
        for (line_float_t line:lines) {
            if (euclideanDist(line.startx, line.starty, line.endx, line.endy) < 10) {
                continue;
            }
            cout << "Line: " << line.startx << "," << line.starty << " - " << line.endx << "," << line.endy << endl;
            cv::line(img, Point(line.startx, line.starty), Point(line.endx, line.endy), Scalar(0, 127, 255), 6);
        }

        cv::imshow("Image", img);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}