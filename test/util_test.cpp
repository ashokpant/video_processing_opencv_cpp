// Created by: Ashok Kumar Pant
// Created on: 7/7/20


void main() {
    vector <Point> corners;
    corners.push_back(Point(1, 1));
    corners.push_back(Point(2, 2));
    corners.push_back(Point(0, 0));
    corners.push_back(Point(4, 4));
    corners.push_back(Point(6, 6));
    vector <vector<Point>> c = combo(corners, 4);
    for (vector <Point> p:c) {
        cout << p.size() << " " << p << endl;
    }
}