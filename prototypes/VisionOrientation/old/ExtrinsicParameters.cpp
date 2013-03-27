#include "ExtrinsicParameters.h"


#ifndef ERROR_SUCCESS
#define ERROR_SUCCESS 0
#endif

#ifndef ERROR_INVALID_PARAMETER
#define ERROR_INVALID_PARAMETER 0
#endif



ExtrinsicParameters::ExtrinsicParameters(){

   ExtrinsicParamMatrix.SetSize(4, 4);
   ExtrinsicParamMatrix.I();

   CurRotationMatrix.SetSize(3,3);
   CurRotationMatrix.I();

   RxRyRzUpToDate = 0;

}

ExtrinsicParameters::~ExtrinsicParameters(){

}

int ExtrinsicParameters::SetRotationMatrix(double Rx1,double Rx2,double Rx3,
										   double Ry1,double Ry2,double Ry3,
										   double Rz1,double Rz2,double Rz3)
{

   ExtrinsicParamMatrix(0,0) = Rx1;
   ExtrinsicParamMatrix(1,0) = Ry1;
   ExtrinsicParamMatrix(2,0) = Rz1;

   ExtrinsicParamMatrix(0,1) = Rx2;
   ExtrinsicParamMatrix(1,1) = Ry2;
   ExtrinsicParamMatrix(2,1) = Rz2;

   ExtrinsicParamMatrix(0,2) = Rx3;
   ExtrinsicParamMatrix(1,2) = Ry3;
   ExtrinsicParamMatrix(2,2) = Rz3;

   RxRyRzUpToDate = 0;
   
   return ERROR_SUCCESS;

}

int ExtrinsicParameters::SetTranlation(double Tx, double Ty, double Tz){

   ExtrinsicParamMatrix(0,3) = Tx;
   ExtrinsicParamMatrix(1,3) = Ty;
   ExtrinsicParamMatrix(2,3) = Tz;

   return ERROR_SUCCESS;
}

const Matrix &ExtrinsicParameters::GetExtrinsicParamMatrix() const {

   return ExtrinsicParamMatrix;

}

void ExtrinsicParameters::GetRxRyRz(double &RotAngleX, double &RotAngleY, double &RotAngleZ){

   if(!RxRyRzUpToDate){
      TheRotationConverter.R2AnglesRxRyRz(CurRotationMatrix, Rx, Ry, Rz);
      RxRyRzUpToDate = 1;
   }

   RotAngleX = Rx;
   RotAngleY = Ry;
   RotAngleZ = Rz;

}