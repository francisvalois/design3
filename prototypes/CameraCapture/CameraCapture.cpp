#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace cv;
using namespace std;


int counter = 0;

void saveImage(Mat&, string&);


void printAttrValue(const char* name, float value) {
	cout << name << ":" << value << endl;
}

int main(int, char**)
{
    VideoCapture cap(1);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);

    cap.set(CV_CAP_PROP_BRIGHTNESS, 0.509803);
    cap.set(CV_CAP_PROP_CONTRAST, 0.196078);
    cap.set(CV_CAP_PROP_SATURATION, 0.176470);
    cap.set(CV_CAP_PROP_GAIN, 0);

    if(!cap.isOpened())
        return -1;

    namedWindow("pic", CV_WINDOW_KEEPRATIO);
    for(;;)
    {
        Mat frame;
        cap >> frame;
        imshow("pic", frame);

		int keyPressed = waitKey(30);
		if (keyPressed == 13 || keyPressed == 10) {
			counter++;
			std::string result;
			std::stringstream sstm;
			sstm << counter << ".png";
			result = sstm.str();
			saveImage(frame, result);
		} else if (keyPressed != 13 && keyPressed >= 0) {
			break;
		}
			
    }

    return 0;
}

void saveImage(Mat &pict, string &filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);

}

