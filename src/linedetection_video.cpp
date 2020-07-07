#include "linedetector.h"

int main(int argc, char **argv) {
    VideoCapture capture = cv::VideoCapture(0);
//    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", 1);
    namedWindow("Image1", 1);
    LineDetector ld = LineDetector(1.5, 20);
    while (true) {
        Mat src;
        bool s = capture.read(src);
        if (!s) {
            break;
        }
        vector<SEGMENT> lines;
        lines.clear();
        Mat src_gray;
        cvtColor(src, src_gray, COLOR_RGB2GRAY);
        ld.lineDetection(src_gray, lines);

        Mat blank = Mat::ones(src.rows, src.cols, CV_8UC1);
        blank += 255;
        Mat blank_color;
        cvtColor(blank, blank_color, COLOR_GRAY2BGR);

        for (size_t i = 0; i < lines.size(); i++) {
            SEGMENT seg = lines.at(i);

            int b = (seg.label * 12337) % 256;
            int g = (seg.label * 24776) % 256;
            int r = (seg.label * 11491) % 256;

            ld.drawArrow(src, &seg, Scalar(b, g, r));
            char cLabel[64];
            sprintf(cLabel, "%d", seg.label);
            Point2i pM;
            pM.x = (seg.x1 + seg.x2) / 2;
            pM.y = (seg.y1 + seg.y2) / 2;
            Point2i pe1;
            double dGap = 15.0;
            if (seg.angle < CV_PI / 1.5f) dGap = 5.0;
            double dAngle = (double) seg.angle;
            double dArrowAng = 89.0;
            pe1.x = cvRound(pM.x - dGap * cos(dArrowAng * CV_PI / 180.0 + dAngle));
            pe1.y = cvRound(pM.y - dGap * sin(dArrowAng * CV_PI / 180.0 + dAngle));
            putText(src, cLabel, pe1, FONT_HERSHEY_PLAIN, 1, Scalar(b, g, r), 1, 8, false);
            line(blank_color, Point2f(seg.x1, seg.y1), Point2f(seg.x2, seg.y2), Scalar(0, 0, 255));
        }


        cv::imshow("Image", src);
        cv::imshow("Image1", blank_color);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}