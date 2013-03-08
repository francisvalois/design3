#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

//#include "SudokuReader.h"

using namespace cv;
using namespace std;

/*SudokuReader sudokuReader;

int main(int, char**) {
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
		if (keyPressed == 13 || keyPressed == 10) {
			sudokuReader.extractNumbers(frame);
			cout << "end of number extraction" << endl;
		} else if (keyPressed != 13 && keyPressed >= 0) {
			break;
		}
	}

	return 0;
}*/

