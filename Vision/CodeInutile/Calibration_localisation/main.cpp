
#include <sstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Calib.h"

using namespace cv;

string i2string(int i) {
	std::ostringstream buffer;
	buffer << i;
	return buffer.str();
}

int main(int, char**) {

	double U = 425;
	double V = 436;
	double X = 0;
	double Y = 0;
	double Z = 0;
	string lPath = "Params.txt";

	Calib calib(&U, &V);
	calib.OnLoadParameters(&lPath);

	calib.OnApplyZhang(&X, &Y, &Z);

	std::cout << "X : "<< X << std::endl;
	std::cout << "Y : "<< Y << std::endl;
	std::cout << "Z : "<< Z << std::endl;
}

