// Created by: Ashok Kumar Pant
// Created on: 6/26/20

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;

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

    };

class IdCardDetector {

    static Mat resize(const Mat &img, int width, int height, bool resizeLarger) {
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


    static bool compareContourArea(const vector<Point> &i1, const vector<Point> &i2) {
        double o1 = cv::contourArea(i1);
        double o2 = cv::contourArea(i2);
        return o1 > o2;
    }

    template<typename T>
    T slice(T &v, int m, int n) {
        T t(n - m + 1);
        std::copy(v.begin() + m, v.begin() + n + 1, t.begin());
        return t;
    }

    template<typename T>
    T erase(T &v, int m, int n) {
        v.erase(v.begin() + m, v.begin() + n + 1);
        return v;
    }

    template<typename T>
    T getTopK(T &v, int k) {
        for (int i = k+1; i < v.size(); i++) {
            v.pop_back();
        }
        return v;
    }

    vector<vector<Point>> sortContoursByContourArea(vector<vector<Point>> contours, int topK) {
        // Descending
        if (topK > 0) {
            topK = std::min(topK, (int) contours.size());
            std::partial_sort(contours.begin(), contours.begin() + topK, contours.end(), compareContourArea);
            return getTopK(contours, topK);
        } else {
            std::sort(contours.begin(), contours.begin(), compareContourArea);
            return contours;
        }
    }

    static bool isValidRect(const Rect &rect, int imWidth, int imHeight) {
        double MIN_QUAD_AREA_RATIO = 0.10;
        return rect.area() > imHeight * imWidth * MIN_QUAD_AREA_RATIO;
    }

    int detectV1(Mat &img, IdCardResult &result) {
        try {
            int oh = img.size().height;
            int ow = img.size().width;
            Mat image = resizeLargestDim(img, 640, true);
            int h = image.size().height;
            int w = image.size().width;
            double scale = (float) ow / (float) w;
            cout << "Image size:(" << ow << "," << oh << ")  scaled: (" << w << "," << h << ")\n";
            Mat gray = Mat();
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, gray, Size(5, 5), 0);
            Mat edged = Mat();
            cv::Canny(gray, edged, 100, 200);
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            cv::findContours(edged, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            contours = sortContoursByContourArea(contours, 5);

            Rect rect;
            vector<Point> polygon;
            for (const vector<Point> &c : contours) {
                vector<Point> cApprox;
                double peri = cv::arcLength(c, true);
                cv::approxPolyDP(c, cApprox, 0.02 * peri, true);
                if (cApprox.size() == 4) {
                    Rect r = cv::boundingRect(cApprox);
                    if (isValidRect(r, w, h)) {
                        polygon = cApprox;
                        rect = r;
                        break;
                    }
                }
            }
            if (!rect.empty()) {
                rect = Rect((int) (rect.x * scale), (int) (rect.y * scale), (int) (rect.width * scale),
                            (int) (rect.height * scale));
                if (!polygon.empty()) {
                    vector<Point> _polygon = vector<Point>();
                    for (const Point &p : polygon) {
                        _polygon.emplace_back((int) (p.x * scale), (int) (p.y * scale));
                    }
                    polygon = _polygon;
                }
                result = IdCardResult();
                result.setRect(rect);
                result.setPolygon(polygon);
                return 1;
            }
            return 0;
        } catch (Exception &e) {
            cerr << e.msg;
            return -1;
        }
    }

public:
    int detect(Mat &img, IdCardResult &result) {
        return detectV1(img, result);
    }
};

int main(int argc, char **argv) {
//    VideoCapture capture = cv::VideoCapture(0);
    VideoCapture capture = cv::VideoCapture("rtsp://192.168.0.10:8080/h264_ulaw.sdp");
    namedWindow("Image", 1);
    IdCardDetector detector = IdCardDetector();
    while (true) {
        Mat img;
        bool s = capture.read(img);
        if (!s) {
            break;
        }
        IdCardResult result;
        int status = detector.detect(img, result);
        if (status > 0) {
            cout << result << std::endl;
            Rect rect = result.getRect();
            if (!rect.empty()) {
                cv::rectangle(img, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height),
                              Scalar(0, 127, 255), 6);
            }
        }
        cv::imshow("Image", img);
        int k = cv::waitKey(100);
        if (k == 27) {
            break;
        }
    }
    return 0;
}