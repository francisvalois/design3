#ifndef EXTRINSIC_PARAMETERS_H
#define EXTRINSIC_PARAMETERS_H


#include "Matrix.h"
#include "RotationConverter.h"

class ExtrinsicParameters{

public:

	ExtrinsicParameters();
	virtual ~ExtrinsicParameters();

   int SetRotationMatrix(double,double,double,double,double,double,double,double,double);
   int SetTranlation(double Tx, double Ty, double Tz);

   const Matrix &GetExtrinsicParamMatrix() const;

   void GetRxRyRz(double &RotAngleX, double &RotAngleY, double &RotAngleZ);

private:
   
   Matrix ExtrinsicParamMatrix;
   Matrix CurRotationMatrix;
   double Rx, Ry, Rz;

   int RxRyRzUpToDate;

   RotationConverter TheRotationConverter;

};

#endif
