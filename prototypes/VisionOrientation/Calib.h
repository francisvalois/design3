#ifndef CALIB_H_
#define CALIB_H_
#include <iostream>
#include "Zhang.h"
#include <string>


class Calibration {
public:
	double P;
	double Q;
	double X;
	double Y;
	double Z;
	//string CalibParamsFileNames[10];
	Calibration( double* U,  double* V); 
	virtual ~Calibration();
	Zhang* zhang;
	void ApplyZhang(double* ioX, double* ioY,double* ioZ);
	void LoadParams(int Params);

private:
	bool fichier;

};

#endif