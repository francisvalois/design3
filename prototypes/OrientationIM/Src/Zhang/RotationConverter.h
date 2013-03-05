#ifndef ROTATION_CONVERTER_H
#define ROTATION_CONVERTER_H

#include "Matrix.h"
#include <iostream>
#include <math.h>

#ifndef PI
#define PI     3.141592653589793
#endif

#ifndef REAL_MAX
#define REAL_MAX 1.7976e+308
#endif

#ifndef REAL_MIN
#define REAL_MIN 2.2251e-308
#endif

#ifndef NUM_PRECISION_DOUBLE
#define NUM_PRECISION_DOUBLE  2.22e-16
#endif

class RotationConverter{

public:

	RotationConverter();
	virtual ~RotationConverter();

   void R2AnglesRxRyRz(const Matrix &R, double &Rx, double &Ry, double &Rz);

   void R2AnglesRxInvRyInvRzInv(const Matrix &R, double &RxInv, double &RyInv, double &RzInv);

   void R2AnglesRzRyRx(const Matrix &R, double &Rz, double &Ry, double &Rx);

   void R2AnglesRzInvRyInvRxInv(const Matrix &R, double &RzInv, double &RyInv, double &RxInv);

private:

   void GetAngle(const double &SinAng, const double &CosAng, double &Ang);
 
};

#endif 
