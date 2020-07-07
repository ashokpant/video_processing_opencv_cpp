// Created by: Ashok Kumar Pant
// Created on: 6/26/20
#include "../include/imgquality.h"
#include "opencv2/highgui.hpp"

using namespace std::chrono;

int main(int argc, char **argv) {
    VideoCapture capture = cv::VideoCapture(0);
//    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", 1);
    while (true) {
        Mat img;
        bool s = capture.read(img);
        if (!s) {
            break;
        }
        double blur = ImgQuality::getBlurScore(img);
        double brightness = ImgQuality::getBrightnessScore(img);
        double glareMin, glareMax;
        ImgQuality::getGlareScore(img, glareMin, glareMax);
        bool glare = glareMin > 0 && glareMax >= 253;
        string txt = format("Blur: %.2f, Brightness: %.2f, Glare min: %.2f, Glare max: %.2f", blur, brightness,
                            glareMin, glareMax);
        string txt1 = format("Blur: %d, Dark: %d, Glare: %d", blur < 80, brightness < 0.20, glare);
        cv::putText(img, txt, Point(20, 50), 1, 1, Scalar(0, 204, 204), 2);
        cv::putText(img, txt1, Point(20, 100), 1, 1, Scalar(0, 204, 204), 2);
        cv::imshow("Image", img);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}
