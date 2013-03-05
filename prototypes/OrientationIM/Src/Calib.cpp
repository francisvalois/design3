#include "Calib.h"

Calibration::Calibration(double* U, double* V) {
	P = *U;
	Q = *V;
	X =0;
	Y =0;
	Z =0;
	zhang = new Zhang();
	fichier = false;
}

Calibration::~Calibration() {
}

void Calibration::LoadParams(int Params) {
	if (zhang->SetParameters(Params))
		fichier = true;
	else
		fichier = false;
}

void Calibration::ApplyZhang(double* ioX, double* ioY,double* ioZ) {

	if (!fichier) {
		zhang->Apply(P, Q,X, Y, Z);
	}

	*ioX = X;
	*ioY = Y;
	*ioZ = Z;

}
