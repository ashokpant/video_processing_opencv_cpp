// Created by: Ashok Kumar Pant
// Created on: 7/16/20

#include <jmorecfg.h>
#include "../include/facedetector.h"
#include "../include/commons.h"

int FaceDetector::detectFacesV1(Mat &img, FaceResult &result, int minSize, bool findLargest, bool findLandmarks) {
    try {

        int oh = img.size().height;
        int ow = img.size().width;
        Mat image = resizeLargestDim(img, 640, true);
        int h = image.size().height;
        int w = image.size().width;
        double scale = (float) ow / (float) w;
        cout << "Image size:(" << ow << "," << oh << ")  scaled: (" << w << "," << h << ")\n";
        Mat gray = Mat();
        if (image.channels() > 2) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        auto start = high_resolution_clock::now();
        vector<Rect> faces;
        faceCascade.detectMultiScale(gray, faces, 1.1, 3, 0, Size(minSize, minSize));
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        cout << "Time - face detect: " << duration.count() << " milliseconds" << endl;
        cout << faces.size() << endl;
        if (faces.size() > 1 && findLargest) {
            Rect biggest = faces[0];
            for (int i = 1; i < faces.size(); i++) {
                if (biggest.area() < faces[i].area()) {
                    biggest = faces[i];
                }
            }
            faces.clear();
            faces.push_back(biggest);
        }
        if (!faces.empty()) {
            for (auto &face : faces) {
                face.x = face.x * scale;
                face.y = face.y * scale;
                face.width = face.width * scale;
                face.height = face.height * scale;
            }
            result.setFaces(faces);

            return 1;
        } else {
            return 0;
        }
    } catch (Exception &e) {
        cerr << e.msg;
        return -1;
    }
}

FaceDetector::FaceDetector(const string& faceCascadeFilename) {
    faceCascade.load(faceCascadeFilename);
    if (faceCascade.empty()) {
        throw std::runtime_error("Failed to load face detector model");
    }
}
