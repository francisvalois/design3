#include "ShowWorkspace.h"

using namespace cv;
using namespace std;

ShowWorkspace::ShowWorkspace(){
	white = cv::Scalar(255, 255, 255);
	blue = cv::Scalar(255, 0, 0);
	black = cv::Scalar(0, 0, 0);
}

void ShowWorkspace::showWorkspace(int table[tableX][tableY]) {
    Mat workspace = Mat(tableX, tableY, CV_8UC3, white);

    for (int y = 0; y < (tableY); y++) {
        for (int x = 0; x < (tableX); x++) {
            if (table[x][y] == 1) {
                colorPixel(workspace, black, x, y);
            }
            if (table[x][y] == 2) {
                colorPixel(workspace, blue, x, y);
            }
        }
    }
    transpose(workspace, workspace);

    showWindowWith("Workspace", workspace);
}

void ShowWorkspace::colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x, y)[0] = color[0];
    mat.at<cv::Vec3b>(x, y)[1] = color[1];
    mat.at<cv::Vec3b>(x, y)[2] = color[2];
}

void ShowWorkspace::showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
    waitKey(0);
}

