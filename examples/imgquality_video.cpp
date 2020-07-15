// Created by: Ashok Kumar Pant
// Created on: 6/26/20
#include "../include/imgquality.h"
#include "opencv2/highgui.hpp"

using namespace std::chrono;

int main(int argc, char **argv) {
    VideoCapture capture = cv::VideoCapture(0);
//    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", WINDOW_GUI_NORMAL);
    while (true) {
        Mat img;
        bool s = capture.read(img);
        if (!s) {
            break;
        }

        double blur = ImgQuality::getBlurScore(img);
        bool hasBlur = blur < 100;

        double brightness = ImgQuality::getBrightnessScore(img);
        bool isDark = brightness < 0.20;
        bool isBright = brightness > 0.80;

        double glare1Min, glare1Max;
        ImgQuality::getGlareScore(img, glare1Min, glare1Max);

        bool hasGlare1 = glare1Min == 0 && glare1Max >= 253;

        double glare2 = ImgQuality::getGlareScore2(img);
        bool hasGlare2 = glare2 > 5000 && glare2 < 8500;

        String txt = format("Blur: %.2f, Brightness: %.2f", blur, brightness);
        String txt1 = format("Glare1 min: %.2f,  Glare1 max: %.2f, Glare2: %.2f", glare1Min, glare1Max, glare2);
        String txt2 = format("Blur: %d, Dark: %d, Bright: %d, Glare1: %d, Glare2: %d", hasBlur, isDark, isBright,
                             hasGlare1, hasGlare2);

        cv::putText(img, txt, Point(10, 50), 1, 1, Scalar(0, 204, 204), 1);
        cv::putText(img, txt1, Point(10, 100), 1, 1, Scalar(0, 204, 204), 1);
        cv::putText(img, txt2, Point(10, 150), 1, 1, Scalar(0, 204, 204), 1);
        cv::imshow("Image", img);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}
