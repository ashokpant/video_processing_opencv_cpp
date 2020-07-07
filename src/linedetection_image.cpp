#include "linedetector.h"

std::string getCmdOption(int argc, char* argv[], const std::string& option, string fallback)
{
    std::string cmd;
    for( int i = 0; i < argc; ++i)
    {
        std::string arg = argv[i];
        if(0 == arg.find(option))
        {
            std::size_t found = arg.find_last_of(option);
            cmd =arg.substr(found + 1);
            return cmd;
        }
    }
    return fallback;
}

/** Main function*/
int main(int argc, char *argv[]) {
    string filename = getCmdOption(argc, argv, "-input=", "");
    string output = getCmdOption(argc, argv, "-output=", "");
    float dt = stof(getCmdOption(argc, argv, "-dt=", "1.5"));
    int lt = stoi(getCmdOption(argc, argv, "-lt=", "20"));

    if (filename.empty()) {
        fprintf(stderr, "Define input file path.\n");
        cout << "Usage: " << argv[0]
             << " -filename /input/file/path.png (optional)-output /output/file/path.txt (optional)-dt distance_threshold (optional)-lt length_threshold"
             << endl;
        return 0;
    }

    LineDetector ld = LineDetector(dt, lt);
    vector<SEGMENT> lines;
    Mat src_old;
    Mat src = imread(filename, 1);
    if (src.cols == 0) {
        fprintf(stderr, "cannot open the input file!");
        return 0;
    }

    Mat src_gray;
    cvtColor(src, src_gray, COLOR_RGB2GRAY);
    lines.clear();
    ld.lineDetection(src_gray, lines);

    FILE *fptr;
    string outfile;
    if (output.empty()) {
        outfile = filename + "_lines.txt";
    } else {
        outfile = output;
    }
    cout << "Writing coordinates of lines to file " << outfile << endl;
    fptr = fopen(outfile.c_str(), "w");
    for (size_t i = 0; i < lines.size(); i++) {
        SEGMENT seg = lines.at(i);
        fprintf(fptr, "%d %.5f %.5f %.5f %.5f %.5f \n", seg.label, seg.angle, seg.x1, seg.y1, seg.x2, seg.y2);
    }
    fclose(fptr);

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

    imshow(filename, src);

    string outimgpath = filename + "_labeled.png";
    imwrite(outimgpath, src);
    cout << outimgpath << " saved. " << endl;
    outimgpath = filename + "_only_lines.png";
    imwrite(outimgpath, blank_color);
    cout << outimgpath << " saved. " << endl;
    cout << "lines.size() = " << lines.size() << endl;

    return 0;
}