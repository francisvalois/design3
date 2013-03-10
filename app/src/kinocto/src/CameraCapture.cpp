#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "SudocubeExtractor.h"

using namespace cv;
using namespace std;

SudocubeExtractor sudocubeReader;

void saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}

/*int main(int, char**) {
    int i = 0;
	VideoCapture cap(1);

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);

	cap.set(CV_CAP_PROP_BRIGHTNESS, 0.509803);
	cap.set(CV_CAP_PROP_CONTRAST, 0.196078);
	cap.set(CV_CAP_PROP_SATURATION, 0.176470);
	cap.set(CV_CAP_PROP_GAIN, 0);

	if (!cap.isOpened())
		return -1;

	namedWindow("pic", CV_WINDOW_KEEPRATIO);
	for (;;) {
		Mat frame;
		cap >> frame;
		imshow("pic", frame);

		int keyPressed = waitKey(30);
		cout << keyPressed << endl;
		if (keyPressed == 13 || keyPressed == 10) {
		    char filename[255];
		    sprintf(filename, "%d.png", i);
			saveImage(frame, filename);
			cout << "end of number extraction" << endl;
			i++;
		} else if (keyPressed != 13 && keyPressed >= 0) {
			break;
		}
	}

	return 0;
}
*/
