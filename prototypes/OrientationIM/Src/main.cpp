#include <sstream>
#include <string>
#include <iostream>

#include "cv.h"
#include "highgui.h"
#include "Calib.h"

using namespace cv;

string i2string(int i) {
	std::ostringstream buffer;
	buffer << i;
	return buffer.str();
}

string CalibParamsFileNames[12] = { "", "ParamsFile/Params1.txt",
		"ParamsFile/Params2.txt", "ParamsFile/Params3.txt",
		"ParamsFile/Params4.txt", "ParamsFile/Params5.txt",
		"ParamsFile/Params6.txt", "ParamsFile/Params7.txt",
		"ParamsFile/Params8.txt", "ParamsFile/Params9.txt",
		"ParamsFile/Params10.txt", "ParamsFile/Params11.txt", };

int main(int, char**) {

	double U = 377;
	double V = 281;
	double X = 0;
	double Y = 0;
	double Z = 0;

	while (true) {

		std::cout << "Entrer U : " ;
		if (std::cin >> U) {
			std::cout << "Entrer V :" ;
			if (std::cin >> V) {
				Calibration calib(&U, &V);
				//calib.LoadParams(&CalibParamsFileNames[1]);
				calib.LoadParams(1);
				calib.ApplyZhang(&X, &Y, &Z);

				std::cout << "\nX : " << X * 100 << "cm"<<std::endl;
				std::cout << "Y : " << Y * 100 << "cm"<<std::endl;
				std::cout << "Z : " << Z * 100 << "cm\n"<<std::endl;

			}
		}

	}
}
