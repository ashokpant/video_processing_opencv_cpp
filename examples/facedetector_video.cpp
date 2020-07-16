// Created by: Ashok Kumar Pant
// Created on: 6/26/20
#include "../include/facedetector.h"

int main(int argc, char **argv) {
    VideoCapture capture = cv::VideoCapture(0);
//    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", 1);
    string faceCascadeFilename = "/home/ashok/Projects/ashok/video_processing_opencv_cpp/resources/lbpcascade_frontalface.xml";
    FaceDetector faceDetector = FaceDetector(faceCascadeFilename);
    while (true) {
        Mat src;
        bool s = capture.read(src);
        if (!s) {
            break;
        }
        Mat img = src.clone();
        auto start = high_resolution_clock::now();
        FaceResult result;
        int success = faceDetector.detect(src, result);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        cout << "Time: " << duration.count() << " milliseconds" << endl;
        cout << success << endl;
        if (success > 0) {
            if (!result.getFaces().empty()) {
                for (const Rect &rect:result.getFaces()) {
                    cv::rectangle(img, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height),
                                  Scalar(0, 127, 255), 4);
                }
            }
        }
        cout << "Result: " << result.toString() << endl;
        cv::imshow("Image", img);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}
