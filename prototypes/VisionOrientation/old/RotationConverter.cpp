#include "RotationConverter.h"



RotationConverter::RotationConverter(){

}

RotationConverter::~RotationConverter(){

}

void RotationConverter::R2AnglesRxRyRz(const Matrix &R, double &Rx, double &Ry, double &Rz){

   double r1 = R(0,0);
   double r2 = R(0,1);
   double r3 = R(0,2);
   double r4 = R(1,0);
   double r5 = R(1,1);
   double r6 = R(1,2);
   double r9 = R(2,2);

   double r3Abs = (r3>0.0 ? r3:-r3);
   if( (1.0 - r3Abs) > NUM_PRECISION_DOUBLE ){

 
      Ry = asin(r3);

      if( (r9>0.0?r9:-r9) > NUM_PRECISION_DOUBLE){
         Rx = atan(-r6/r9);
      }else{
         Rx = PI/2.0;
         if(r6 > 0.0) Rx = -Rx;
      }

      if( (r1>0.0?r1:-r1) > NUM_PRECISION_DOUBLE){
         Rz = atan(-r2/r1);
      }else{
         Rz = PI/2.0;
         if(r2 > 0.0) Rz = -Rz;
      }


      double RxAltern[2];
      double RyAltern[2];
      double RzAltern[2];


      RxAltern[0] = Rx;
      RyAltern[0] = Ry;
      RzAltern[0] = Rz;

      if(Rx > 0.0) RxAltern[1] =  Rx - PI;
      else         RxAltern[1] =  Rx + PI;
      if(Ry > 0.0) RyAltern[1] =  PI - Ry;
      else         RyAltern[1] = -PI - Ry;
      if(Rz > 0.0) RzAltern[1] =  Rz - PI;
      else         RzAltern[1] =  Rz + PI;


        enum {MaxIsRx, MaxIsRy, MaxIsRz};

      int Max = MaxIsRx;
      double MaxVal = (Rx>0.0?Rx:-Rx);
   
      if( (Ry>0.0?Ry:-Ry) > MaxVal){
         Max = MaxIsRy;
         MaxVal = (Ry>0.0?Ry:-Ry);
      }
      if( (Rz>0.0?Rz:-Rz) > MaxVal ){
         Max = MaxIsRz;
         MaxVal = (Rz>0.0?Rz:-Rz);
      }

      Matrix TestMatrix(3,3);
      Matrix RxTestMatrix(3,3);
      Matrix RyTestMatrix(3,3);
      Matrix RzTestMatrix(3,3);
   
      double ErrorHypothesis[2];  
   
      double SinRx, SinRy, SinRz;
      double CosRx, CosRy, CosRz;

     
      for(int Hypothesis = 0; Hypothesis<2; Hypothesis++){

         switch(Max){

         case MaxIsRx :

            SinRx = sin(RxAltern[Hypothesis]);
            CosRx = cos(RxAltern[Hypothesis]);

    
            if( (SinRx>0.0?SinRx:-SinRx) < NUM_PRECISION_DOUBLE ){
               CosRy =  r9/CosRx;
            }else{
               CosRy = -r6/SinRx;
            }
            SinRy = r3;

            GetAngle(SinRy, CosRy, RyAltern[Hypothesis]);

            SinRz = -r2/CosRy;
            CosRz =  r1/CosRy;

            GetAngle(SinRz, CosRz, RzAltern[Hypothesis]);

            break;

         case MaxIsRy :

  
            SinRy = sin(RyAltern[Hypothesis]);
            CosRy = cos(RyAltern[Hypothesis]);

 
            SinRz = -r2/CosRy;
            CosRz =  r1/CosRy;

            GetAngle(SinRz, CosRz, RzAltern[Hypothesis]);


            SinRx = -r6/CosRy;
            CosRx =  r9/CosRy;

            GetAngle(SinRx, CosRx, RxAltern[Hypothesis]);

            break;

         case MaxIsRz :

            
            SinRz = sin(RzAltern[Hypothesis]);
            CosRz = cos(RzAltern[Hypothesis]);
            if( (SinRz>0.0?SinRz:-SinRz) < NUM_PRECISION_DOUBLE ){
               CosRy =  r1/CosRz;
            }else{
               CosRy = -r2/SinRz;
            }
            SinRy = r3;

            GetAngle(SinRy, CosRy, RyAltern[Hypothesis]);
            SinRx = -r6/CosRy;
            CosRx =  r9/CosRy;

            GetAngle(SinRx, CosRx, RxAltern[Hypothesis]);

            break;

         }

         RxTestMatrix.I();
         RyTestMatrix.I();
         RzTestMatrix.I();

         RxTestMatrix(1,1) = cos(RxAltern[Hypothesis]);
         RxTestMatrix(2,2) = RxTestMatrix(1,1);
         RxTestMatrix(2,1) = sin(RxAltern[Hypothesis]);
         RxTestMatrix(1,2) = -RxTestMatrix(2,1);
         RyTestMatrix(0,0) = cos(RyAltern[Hypothesis]);
         RyTestMatrix(2,2) = RyTestMatrix(0,0);
         RyTestMatrix(0,2) = sin(RyAltern[Hypothesis]);
         RyTestMatrix(2,0) = -RyTestMatrix(0,2);
         RzTestMatrix(0,0) = cos(RzAltern[Hypothesis]);
         RzTestMatrix(1,1) = RzTestMatrix(0,0);
         RzTestMatrix(1,0) = sin(RzAltern[Hypothesis]);
         RzTestMatrix(0,1) = -RzTestMatrix(1,0);

         TestMatrix = RxTestMatrix*RyTestMatrix*RzTestMatrix;

         TestMatrix -= R;

         double Err = 0.0;
         for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
               Err += TestMatrix(i,j)*TestMatrix(i,j);
            }
         }
         ErrorHypothesis[Hypothesis] = Err;
      
      }

      double ErrorDiff = ErrorHypothesis[0]-ErrorHypothesis[1];

      if( (ErrorDiff>0.0?ErrorDiff:-ErrorDiff) < NUM_PRECISION_DOUBLE ){

         double SumAngles[2] = {0.0, 0.0};
         for(int i=0; i<2; i++){
            SumAngles[i] += (RxAltern[i]>0.0)?RxAltern[i]:-RxAltern[i];
            SumAngles[i] += (RyAltern[i]>0.0)?RyAltern[i]:-RyAltern[i];
            SumAngles[i] += (RzAltern[i]>0.0)?RzAltern[i]:-RzAltern[i];
         }
         int GoodAlt = 0;
         if(SumAngles[1] < SumAngles[0]) GoodAlt = 1;
         Rx = RxAltern[GoodAlt];
         Ry = RyAltern[GoodAlt];
         Rz = RzAltern[GoodAlt];

      }else{
         
         //One of the two hypothesis is good. Let's choose the best one.
         int GoodAlt = 0;
         if(ErrorHypothesis[1] < ErrorHypothesis[0]) GoodAlt = 1;
         Rx = RxAltern[GoodAlt];
         Ry = RyAltern[GoodAlt];
         Rz = RzAltern[GoodAlt];

      }

   }else{
      
     
      if(r3>0.0){
         Ry =  PI/2.0;
         if( (r5>0.0?r5:-r5) > NUM_PRECISION_DOUBLE){
            Rx = atan2(r4,r5);
         }else{
            if( r4>0.0) Rx =  PI/2.0;
            else        Rx = -PI/2.0;
         }
      }else{
         Ry = -PI/2.0;
         if( (r5>0.0?r5:-r5) > NUM_PRECISION_DOUBLE){
            Rx = atan2(-r4,r5);
         }else{
            if(-r4>0.0) Rx =  PI/2.0;
            else        Rx = -PI/2.0;
         }
      }

      Rz = 0.0;
   
   }

}


void RotationConverter::R2AnglesRxInvRyInvRzInv(const Matrix &R, double &RxInv, double &RyInv, double &RzInv){

   R2AnglesRxRyRz(R, RxInv, RyInv, RzInv);
   RxInv = -RxInv;
   RyInv = -RyInv;
   RzInv = -RzInv;

}

void RotationConverter::R2AnglesRzRyRx(const Matrix &R, double &Rz, double &Ry, double &Rx){

   Matrix RTemp = R;

     RTemp.Transpose();

   R2AnglesRxInvRyInvRzInv(RTemp, Rx, Ry, Rz);


}

void RotationConverter::R2AnglesRzInvRyInvRxInv(const Matrix &R, double &RzInv, double &RyInv, double &RxInv){

   Matrix RTemp = R;

   RTemp.Transpose();

   R2AnglesRxRyRz(RTemp, RxInv, RyInv, RzInv);


}

void RotationConverter::GetAngle(const double &SinAng, const double &CosAng, double &Ang){

   double SinAngAbs = SinAng>0.0?SinAng:-SinAng;
   double CosAngAbs = CosAng>0.0?CosAng:-CosAng;

   if( CosAngAbs < NUM_PRECISION_DOUBLE ){
      if(SinAng > 0.0) Ang =  PI/2.0;
      else             Ang = -PI/2.0;
   }else{
      Ang = atan(SinAngAbs/CosAngAbs);
      if(CosAng > 0.0){
         if(SinAng < 0.0) Ang = -Ang;
      }else{
         if(SinAng > 0.0) Ang =  PI - Ang;
         else             Ang = -PI + Ang;
      }
   }

}
