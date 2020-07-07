// Created by: Ashok Kumar Pant
// Created on: 6/26/20
#include "carddetector.h"

float PI = 3.141592653589793238463;

static bool compareArcLength(const vector<Point> &i1, const vector<Point> &i2) {
    double o1 = cv::arcLength(i1, true);
    double o2 = cv::arcLength(i2, true);
    return o1 > o2;
}

template<typename T>
T getTopK(T &v, int k) {
    for (int i = k + 1; i < v.size(); i++) {
        v.pop_back();
    }
    return v;
}

template<typename ... Args>
std::string string_format(const std::string &format, Args ... args) {
    size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
    if (size <= 0) { throw std::runtime_error("Error during formatting."); }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

vector<vector<Point>> sortContoursByArcLength(vector<vector<Point>> contours, int topK) {
    // Descending
    if (topK > 0) {
        topK = std::min(topK, (int) contours.size());
        std::partial_sort(contours.begin(), contours.begin() + topK, contours.end(), compareArcLength);
        return getTopK(contours, topK);
    } else {
        std::sort(contours.begin(), contours.begin(), compareArcLength);
        return contours;
    }
}

static bool comparePointByX(const Point &i1, Point &i2) {
    // Ascending
    if (i1.x != i2.x) {
        return i1.x < i2.x;
    } else {
        return i1.y < i2.y;
    }
}

static bool comparePointByY(const Point &i1, Point &i2) {
    // Ascending
    if (i1.y != i2.y) {
        return i1.y < i2.y;
    } else {
        return i1.x < i2.x;
    }
}

vector<Point> sortPoints(vector<Point> points, bool sortByX) {
    //Ascending
    if (sortByX)
        std::sort(points.begin(), points.begin(), comparePointByX);
    else
        std::sort(points.begin(), points.begin(), comparePointByY);
    return points;
}

double dist(const Point &point1, const Point &point2) {
    Mat d = Mat(1, 2, CV_32F);
    d.at<int>(0, 0) = point1.x - point2.x;
    d.at<int>(0, 1) = point1.y - point2.y;
    return cv::norm(d);
}

bool predicate(const vector<Point> &corners, const Point &point) {
    int MIN_DIST = 20;
    for (const Point &p : corners) {
        if (dist(p, point) < MIN_DIST) {
            return false;
        }
    }
    return true;
}

vector<Point> filterCorners(vector<Point> corners) {
    // Filters corners that are within min_dist of others
    vector<Point> filtered = vector<Point>();
    for (const Point &p : corners) {
        if (predicate(filtered, p)) {
            filtered.push_back(p);
        }
    }
    return filtered;
}


array<int, 5> sortLineSegment(int x1, int y1, int x2, int y2, int angle, bool sortByX) {
    //Ascending
    array<int, 5> out{};
    if (sortByX) {
        if (x1 > x2)
            out = {x2, y2, x1, y1, angle};
        else if (x1 < x2)
            out = {x1, y1, x2, y2, angle};
        else if (y1 > y2)
            out = {x2, y2, x1, y1, angle};
        else
            out = {x1, y1, x2, y2, angle};
    } else {
        if (y1 > y2)
            out = {x2, y2, x1, y1, angle};
        else if (y1 < y2)
            out = {x1, y1, x2, y2, angle};
        else if (x1 > x2)
            out = {x2, y2, x1, y1, angle};
        else
            out = {x1, y1, x2, y2, angle};
    }

    return out;
}

array<int, 5> sortLineSegment(std::array<int, 5> line, bool sortByX) {
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];
    int angle = line[4];
    return sortLineSegment(x1, y1, x2, y2, angle, sortByX);
}

double average(const vector<int> &x) {
    double s = 0;
    for (int e:x) {
        s += e;
    }
    s /= x.size();
    return s;
}

vector<Point> getCornersV1(const Mat &img, const vector<std::array<int, 5>> &lines, int imWidth, int imHeight) {
    // line: [x1, y1, x2, y2, angle]
    namedWindow("Vertical", 1);
    namedWindow("Horizontal", 1);
    namedWindow("Card", 1);
    Mat img1 = img.clone();
    Mat img2 = img.clone();
    Mat img3 = img.clone();
    Mat img4 = img.clone();
    Size imSize = Size(imWidth, imHeight);
    vector<Point> corners = vector<Point>();
    if (!lines.empty()) {
        //        separate out the horizontal and vertical lines, and draw them back onto separate canvases
        Mat horizontalLinesCanvas = Mat::zeros(imSize, CV_8U);
        Mat verticalLinesCanvas = Mat::zeros(imSize, CV_8U);

        for (auto line: lines) {
            int x1 = line[0];
            int y1 = line[1];
            int x2 = line[2];
            int y2 = line[3];
            int angle = line[4];

            if (abs(x2 - x1) > abs(y2 - y1)) {
                line = sortLineSegment(x1, y1, x2, y2, angle, true);
                x1 = line[0];
                y1 = line[1];
                x2 = line[2];
                y2 = line[3];
                cout << string_format("Hor: %d,-%d, , %d-%d, angle=%d", x1, y1, x2, y2, angle) << endl;
                cv::putText(horizontalLinesCanvas, "A", Point(max(x1 - 5, 0), y1), FONT_HERSHEY_SIMPLEX, 1,
                            Scalar(255, 0, 255));
                cv::line(horizontalLinesCanvas, Point(max(x1 - 5, 0), y1),
                         Point(min(x2 + 5, imSize.width - 1), y2), Scalar(255, 255, 255), 2);
            }
            {
                line = sortLineSegment(x1, y1, x2, y2, angle, false);
                x1 = line[0];
                y1 = line[1];
                x2 = line[2];
                y2 = line[3];
                cout << string_format("Ver: %d,-%d, , %d-%d, angle=%d", x1, y1, x2, y2, angle) << endl;
                cv::putText(verticalLinesCanvas, "A", Point(max(x1 - 5, 0), y1), FONT_HERSHEY_PLAIN, 1,
                            Scalar(255, 0, 255));
                cv::line(verticalLinesCanvas, Point(max(x1 - 5, 0), y1),
                         Point(min(x2 + 5, imSize.height - 1), y2), Scalar(255, 255, 255), 2);
            }
        }
        cv::imshow("Vertical", verticalLinesCanvas);
        cv::imshow("Horizontal", horizontalLinesCanvas);
        vector<std::array<int, 4>> selectedLines = vector<std::array<int, 4>>();

        // find the horizontal lines (connected-components -> bounding boxes -> final lines)
        vector<vector<Point>> contours;
        Mat hierarchy = Mat();
        cv::findContours(horizontalLinesCanvas, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        contours = sortContoursByArcLength(contours, 2);
        horizontalLinesCanvas = Mat::zeros(imSize, CV_8U);
        for (vector<Point> points : contours) {
            cout << points.size() << endl;
            points = sortPoints(points, true);
            int minX = (int) points[0].x;
            int maxX = (int) points[points.size() - 1].x;

            vector<int> minY = vector<int>();
            vector<int> maxY = vector<int>();
            for (const Point &p : points) {
                if ((int) p.x == minX) {
                    minY.push_back((int) p.y);
                }
                if ((int) p.x == maxX) {
                    maxY.push_back((int) p.y);
                }
            }
            int leftY = (int) average(minY);
            int rightY = (int) average(maxY);
            selectedLines.push_back({minX, leftY, maxX, rightY});
            cv::putText(img4, "A", Point(minX, leftY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(255, 0, 255));
            cv::putText(img4, "B", Point(maxX, rightY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(255, 0, 255));
            cv::line(img4, Point(minX, leftY), Point(maxX, rightY), Scalar(255, 255, 255), 2);
            cv::line(horizontalLinesCanvas, Point(minX, leftY), Point(maxX, rightY), Scalar(1, 1, 1), 1);
            corners.push_back(Point(minX, leftY));
            corners.push_back(Point(maxX, rightY));
            cout << string_format("H: %d,- %d, , %d - %d", minX, leftY, maxX, rightY) << endl;
        }

        // find the vertical lines (connected-components -> bounding boxes -> final lines)
        contours.clear();
        hierarchy = Mat();
        cv::findContours(verticalLinesCanvas, contours, hierarchy, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE);
        contours = sortContoursByArcLength(contours, 2);
        verticalLinesCanvas = Mat::zeros(imSize, CV_8U);
        for (vector<Point> points : contours) {
            points = sortPoints(points, false);
            for (Point p:points) {
                cout << p.x << " " << p.y << endl;
            }
            int minY = (int) points[0].y;
            int maxY = (int) points[points.size() - 1].y;
            vector<int> minX = vector<int>();
            vector<int> maxX = vector<int>();

            for (const Point &p : points) {
                if ((int) p.y == minY) {
                    minX.push_back((int) p.x);
                }
                if ((int) p.y == maxY) {
                    maxX.push_back((int) p.x);
                }
            }
            int topX = (int) average(minX);
            int bottomX = (int) average(maxX);
            selectedLines.push_back({topX, minY, bottomX, maxY});

            cv::line(verticalLinesCanvas, Point(topX, minY), Point(bottomX, maxY), Scalar(1, 1, 1), 1);
            cv::putText(img4, "A", Point(topX, minY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(0, 255, 255));
            cv::putText(img4, "B", Point(bottomX, maxY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(0, 255, 255));
            cv::line(img4, Point(topX, minY), Point(bottomX, maxY), Scalar(255, 255, 255), 2);
            corners.push_back(Point(topX, minY));
            corners.push_back(Point(bottomX, maxY));
            cout << string_format("V: %d,- %d, , %d - %d", topX, minY, bottomX, maxY) << endl;
        }
        cv::imshow("Vertical1", verticalLinesCanvas);
        cv::imshow("Horizontal1", horizontalLinesCanvas);
        cv::imshow("Card", img4);
        Mat hv = Mat();
        cv::add(horizontalLinesCanvas, verticalLinesCanvas, hv);
        for (int i = 0; i < hv.rows; i++) {
            for (int j = 0; j < hv.cols; j++) {
                if (hv.at<int>(i, j) == 2) {
                    corners.push_back(Point(i, j));
                }
            }
        }
        corners = filterCorners(corners);
    }
    return corners;
}

//Better
vector<Point>
getCornersWithGivenLineAngle(const Mat &img, const vector<std::array<int, 5>> &lines, int imWidth, int imHeight) {
    // line: [x1, y1, x2, y2, angle]
    namedWindow("Vertical", 1);
    namedWindow("Horizontal", 1);
    namedWindow("Card", 1);
    Mat img1 = img.clone();
    Mat img2 = img.clone();
    Mat img3 = img.clone();
    Mat img4 = img.clone();
    Size imSize = Size(imWidth, imHeight);
    vector<Point> corners = vector<Point>();
    if (!lines.empty()) {
        //        separate out the horizontal and vertical lines, and draw them back onto separate canvases
        Mat horizontalLinesCanvas = Mat::zeros(imSize, CV_8U);
        Mat verticalLinesCanvas = Mat::zeros(imSize, CV_8U);
        for (auto line: lines) {
            int x1 = line[0];
            int y1 = line[1];
            int x2 = line[2];
            int y2 = line[3];
            int angle = line[4];

            if (angle > 180) {
                angle = 360 - angle;
                int tx1 = x1;
                int ty1 = y1;
                x1 = x2;
                y1 = y2;
                x2 = tx1;
                y2 = ty1;
            }
            if ((angle >= 0 && angle <= 5) || (angle >= 175 && angle <= 180)) {
                cout << string_format("Hor: %d,-%d, , %d-%d, angle=%d", x1, y1, x2, x2, angle) << endl;
                cv::putText(horizontalLinesCanvas, "A", Point(max(x1 - 5, 0), y1), FONT_HERSHEY_SIMPLEX, 1,
                            Scalar(255, 0, 255));
                cv::line(horizontalLinesCanvas, Point(max(x1 - 5, 0), y1),
                         Point(min(x2 + 5, imSize.width - 1), y2), Scalar(255, 255, 255), 2);
            } else if (angle > 85 && angle < 95) {
                cout << string_format("Ver: %d,-%d, , %d-%d, angle=%d", x1, y1, x2, x2, angle) << endl;
                cv::putText(verticalLinesCanvas, "A", Point(max(x1 - 5, 0), y1), FONT_HERSHEY_PLAIN, 1,
                            Scalar(255, 0, 255));
                cv::line(verticalLinesCanvas, Point(max(x1 - 5, 0), y1),
                         Point(min(x2 + 5, imSize.height - 1), y2), Scalar(255, 255, 255), 2);
            }
        }

        cv::imshow("Vertical", verticalLinesCanvas);
        cv::imshow("Horizontal", horizontalLinesCanvas);
        vector<std::array<int, 4>> selectedLines = vector<std::array<int, 4>>();

        // find the horizontal lines (connected-components -> bounding boxes -> final lines)
        vector<vector<Point>> contours;
        Mat hierarchy = Mat();
        cv::findContours(horizontalLinesCanvas, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        contours = sortContoursByArcLength(contours, 2);
        horizontalLinesCanvas = Mat::zeros(imSize, CV_8U);
        for (vector<Point> points : contours) {
            cout << points.size() << endl;
            points = sortPoints(points, true);
            int minX = (int) points[0].x;
            int maxX = (int) points[points.size() - 1].x;

            vector<int> minY = vector<int>();
            vector<int> maxY = vector<int>();
            for (const Point &p : points) {
                if ((int) p.x == minX) {
                    minY.push_back((int) p.y);
                }
                if ((int) p.x == maxX) {
                    maxY.push_back((int) p.y);
                }
            }
            int leftY = (int) average(minY);
            int rightY = (int) average(maxY);
            selectedLines.push_back({minX, leftY, maxX, rightY});
            cv::putText(img4, "A", Point(minX, leftY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(255, 0, 255));
            cv::putText(img4, "B", Point(maxX, rightY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(255, 0, 255));
            cv::line(img4, Point(minX, leftY), Point(maxX, rightY), Scalar(255, 255, 255), 2);
            cv::line(horizontalLinesCanvas, Point(minX, leftY), Point(maxX, rightY), Scalar(1, 1, 1), 1);
            corners.push_back(Point(minX, leftY));
            corners.push_back(Point(maxX, rightY));
            cout << string_format("H: %d,- %d, , %d - %d", minX, leftY, maxX, rightY) << endl;
        }

        // find the vertical lines (connected-components -> bounding boxes -> final lines)
        contours.clear();
        hierarchy = Mat();
        cv::findContours(verticalLinesCanvas, contours, hierarchy, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE);
        contours = sortContoursByArcLength(contours, 2);
        verticalLinesCanvas = Mat::zeros(imSize, CV_8U);
        for (vector<Point> points : contours) {
            points = sortPoints(points, false);
            for (Point p:points) {
                cout << p.x << " " << p.y << endl;
            }
            int minY = (int) points[0].y;
            int maxY = (int) points[points.size() - 1].y;
            vector<int> minX = vector<int>();
            vector<int> maxX = vector<int>();

            for (const Point &p : points) {
                if ((int) p.y == minY) {
                    minX.push_back((int) p.x);
                }
                if ((int) p.y == maxY) {
                    maxX.push_back((int) p.x);
                }
            }
            int topX = (int) average(minX);
            int bottomX = (int) average(maxX);
            selectedLines.push_back({topX, minY, bottomX, maxY});

            cv::line(verticalLinesCanvas, Point(topX, minY), Point(bottomX, maxY), Scalar(1, 1, 1), 1);
            cv::putText(img4, "A", Point(topX, minY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(0, 255, 255));
            cv::putText(img4, "B", Point(bottomX, maxY), FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(0, 255, 255));
            cv::line(img4, Point(topX, minY), Point(bottomX, maxY), Scalar(255, 255, 255), 2);
            corners.push_back(Point(topX, minY));
            corners.push_back(Point(bottomX, maxY));
            cout << string_format("V: %d,- %d, , %d - %d", topX, minY, bottomX, maxY) << endl;
        }
        cv::imshow("Vertical1", verticalLinesCanvas);
        cv::imshow("Horizontal1", horizontalLinesCanvas);
        cv::imshow("Card", img4);
        Mat hv = Mat();
        cv::add(horizontalLinesCanvas, verticalLinesCanvas, hv);
        for (int i = 0; i < hv.rows; i++) {
            for (int j = 0; j < hv.cols; j++) {
                if (hv.at<int>(i, j) == 2) {
                    corners.push_back(Point(i, j));
                }
            }
        }
        corners = filterCorners(corners);
    }
    return corners;
}

vector<std::array<int, 5>> convert(const vector<SEGMENT> &segments) {

    vector<std::array<int, 5>> lines;

    for (SEGMENT line:segments) {
        std::array<int, 5> l{};
        l[0] = line.x1;
        l[1] = line.y1;
        l[2] = line.x2;
        l[3] = line.y2;
        l[4] = line.angle * 180 / PI;
        lines.push_back(l);
    }
    return lines;
}

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

int getAngle(Point pt1, Point pt2, Point pt3) {

    //    Returns the angle between the line segment from p2 to p1
    //    and the line segment from p2 to p3 in degrees
    double dx21 = pt2.x - pt1.x;
    double dy21 = pt2.y - pt1.y;

    double dx31 = pt3.x - pt1.x;
    double dy31 = pt3.y - pt1.y;
    double m12 = sqrt(dx21 * dx21 + dy21 * dy21);
    double m13 = sqrt(dx31 * dx31 + dy31 * dy31);
    double div = (m12 * m13);
    if (div == 0) {
        return 360;
    }
    double numerator = (dx21 * dx31 + dy21 * dy31);
    double x = numerator / div;
    if (x < -1) {
        x = -1;
    } else if (x > 1) {
        x = 1;
    }
    double theta = acos(x);
    double degree = (theta * 180.0 / PI);
    return (int) degree;
}

double angleRange(vector<Point> points) {
    Point tl = points[0];
    Point tr = points[1];
    Point br = points[2];
    Point bl = points[3];

    double ura = getAngle(tl, tr, br);
    double ula = getAngle(bl, tl, tr);
    double lra = getAngle(tr, br, bl);
    double lla = getAngle(br, bl, tl);

    double min = ura;
    double max = ura;

    for (double a : {ula, lra, lla}) {
        if (a < min) {
            min = a;
        }
        if (a > max) {
            max = a;
        }
    }
    return max - min;
}

static bool compareContourAngleRange(const vector<Point> &i1, const vector<Point> &i2) {
    double o1 = angleRange(i1);
    double o2 = angleRange(i2);
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

vector<vector<Point>> sortContoursByAngleRange(vector<vector<Point>> contours, int topK) {
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


template<typename T>
vector<Point> _comb(const T &c, int combo) {
    vector<Point> combination;
    int n = c.size();
    for (int i = 0; i < n; ++i) {
        if ((combo >> i) & 1)
            combination.push_back(c[i]);
        cout << c[i] << endl;
    }
    return combination;
}

template<typename T>
vector<vector<Point>> combo(const T &c, int k) {
    vector<vector<Point>> combinations;
    int n = c.size();
    int combo = (1 << k) - 1;       // k bit sets
    while (combo< 1 << n) {

        combinations.push_back(_comb(c, combo));

        int x = combo & -combo;
        int y = combo + x;
        int z = (combo & ~y);
        combo = z / x;
        combo >>= 1;
        combo |= y;
    }
    return combinations;
}

//    public List<Point> orderPoints(List<Point> pts) {
//        List<Point> points = sortPoints(pts, true);
//        List<Point> leftMost = Arrays.asList(points.get(0), points.get(1));
//        List<Point> rightMost = Arrays.asList(points.get(points.size() - 1), points.get(points.size() - 2));
//        leftMost = sortPoints(leftMost, false);
//        Point tl = leftMost.get(0);
//        Point bl = leftMost.get(1);
//        double[][] dists = cdist(Collections.singletonList(tl), rightMost);
//        Point br;
//        Point tr;
//        if (dists[0][0] > dists[0][1]) {
//            br = rightMost.get(0);
//            tr = rightMost.get(1);
//        } else {
//            br = rightMost.get(1);
//            tr = rightMost.get(0);
//        }
//        return Arrays.asList(tl, bl, br, tr);
//    }
vector<Point> orderPoints(vector<Point> &pts) {
    vector<Point> pts1 = sortPoints(pts, true);
    vector<Point> leftmost = {pts1[0], pts1[1]};
    vector<Point> rightMost = {pts[pts1.size() - 1], pts[pts1.size() - 2]};
    leftmost = sortPoints(leftmost, false);
    rightMost = sortPoints(rightMost, false);
    Point tl = leftmost[0];
    Point bl = leftmost[1];
    Point tr = rightMost[0];
    Point br = rightMost[1];
    vector<Point> ordered = {tl, bl, tr, br};
    return ordered;
}

//    List<MatOfPoint> sortContoursByAngleRange(List<MatOfPoint> contours, int topK) {
//        // Ascending
//        Collections.sort(contours, new Comparator<MatOfPoint>() {
//            @Override
//            public int compare(MatOfPoint o1, MatOfPoint o2) {
//                double a1 = angleRange(o1);
//                double a2 = angleRange(o2);
//                return Double.compare(a1, a2);
//            }
//        });
//        if (topK > 0) {
//            contours = contours.subList(0, Math.min(topK, contours.size()));
//        }
//        return contours;
//    }


vector<vector<Point>> getQuadrilateralPoints(const vector<Point> &corners) {
    vector<vector<Point>> quadPoints;
    if (corners.size() >= 4) {
        vector<vector<Point>> quads;
        for (vector<Point> pts:combo(corners, 4)) {
            pts = orderPoints(pts);
            cout << pts << endl;
            quads.push_back(pts);
        }
        quads = sortContoursByContourArea(quads, 3);
        quadPoints = sortContoursByAngleRange(quads, 0);
    }
    return quadPoints;
}
//public List<List<Point>> getQuadrilateralPoints(List<Point> corners) {
//    // Get valid quadrilateral points
//    List<List<Point>> contours = new ArrayList<>();
//    if (corners.size() >= 4) {
//        List<MatOfPoint> quads = new ArrayList<>();
//        for (Set<Point> pts : Sets.combinations(ImmutableSet.copyOf(corners), 4)) {
//            List<Point> points = orderPoints(new ArrayList<>(pts));
//            MatOfPoint qp = new MatOfPoint();
//            qp.fromList(points);
//            quads.add(qp);
//        }
//        quads = sortContoursByContourArea(quads, 3);
//        quads = sortContoursByAngleRange(quads, 0);
//        for (MatOfPoint p : quads) {
//            contours.add(p.toList());
//        }
//    }
//    return contours;
//}

int IdCardDetector::detectV1(Mat &img, IdCardResult &result) {
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

int IdCardDetector::detectV3(Mat &img, IdCardResult &result) {
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

        auto start = high_resolution_clock::now();
        vector<array<int, 5>> lines;
        lineDetector.detect(gray, lines);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        cout << "Time - line detect: " << duration.count() << " milliseconds" << endl;

        start = high_resolution_clock::now();
        vector<Point> corners = getCornersWithGivenLineAngle(image, lines, w, h);
        stop = high_resolution_clock::now();
        duration = duration_cast<milliseconds>(stop - start);
        cout << "Time - corner detect: " << duration.count() << " milliseconds" << endl;

        if (corners.empty()) {
            return 0;
        }

        for (const Point &p:corners) {
            cv::circle(img, p, 5, Scalar(255, 127, 0), 6);
        }
        cv::imshow("Corners", img);

        for (std::array<int, 5> line:lines) {
            cv::line(img, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 127, 255), 6);
        }
        cv::imshow("Lines", img);

        start = high_resolution_clock::now();
        vector<vector<Point>> contours = getQuadrilateralPoints(corners);
        stop = high_resolution_clock::now();
        duration = duration_cast<milliseconds>(stop - start);
        cout << "Time - quad compute: " << duration.count() << " milliseconds" << endl;
        Rect rect;
        vector<Point> polygon;;
        for (const vector<Point> &contour : contours) {
            Rect r = cv::boundingRect(contour);
            if (isValidRect(r, w, h)) {
                rect = r;
                rect = Rect((int) (rect.x * scale), (int) (rect.y * scale), (int) (rect.width * scale),
                            (int) (rect.height * scale));
                for (const Point &p : contour) {
                    polygon.push_back(Point(p.x * scale, p.y * scale));
                }
                break;
            }
        }

        if (!rect.empty()) {
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

IdCardDetector::IdCardDetector() {
    lineDetector = LineDetector(1.5, 20);
}

