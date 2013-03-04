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

 /*string CalibParamsFileNames[10] = { "", "ParamsFile/Params1.txt",
		"ParamsFile/Params2.txt", "ParamsFile/Params3.txt",
	"ParamsFile/Params4.txt", "ParamsFile/Params5.txt",
	"ParamsFile/Params6.txt", "ParamsFile/Params7.txt",
	"ParamsFile/Params8.txt", "ParamsFile/Params9.txt", };
*/
int main(int, char**) {

	double U = 377;
	double V = 281;
	double X = 0;
	double Y = 0;
	double Z = 0;

	while (true) {
		std::cout << "Entrer x: " ;
		if (std::cin >> U) {
			std::cout << "Entrer y :" ;
			if (std::cin >> V) {
				Calibration calib(&U, &V);
			//calib.LoadParams();
				//calib.LoadParams(2);
				calib.ApplyZhang(&X, &Y, &Z);

				std::cout << "Les parametres en (x,y,z) sont :" ;

				std::cout << "\nX : " << X * 100 << "cm"<<std::endl;
				std::cout << "Y : " << Y * 100 << "cm"<<std::endl;
				std::cout << "Z : " << Z * 100 << "cm\n"<<std::endl;

			}
		}

	}
}

