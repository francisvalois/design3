#include "DistorsionConverter.h"
#include <math.h>
#include <float.h>


#ifndef ERROR_SUCCESS
#define ERROR_SUCCESS 0
#endif

#ifndef RU_INF
#define RU_INF 1.0e50
#endif

DistorsionConverter::DistorsionConverter(){

}

DistorsionConverter::~DistorsionConverter(){

}

int DistorsionConverter::AddRadialDistortion(double k1, double k2, const double Xu, const double Yu, double &Xd, double &Yd){

   double Ru2 = Xu*Xu + Yu*Yu;

   Xd = Xu*(1.0 + Ru2*(k1 + k2*Ru2));
   Yd = Yu*(1.0 + Ru2*(k1 + k2*Ru2));

   return ERROR_SUCCESS;
}

int DistorsionConverter::RemoveRadialDistortion(double k1, double k2, const double Xd, const double Yd, double &Xu, double &Yu, double Precision){


   if((k1>0.0?k1:-k1) < DBL_EPSILON){
      k1 = 0.0;
   }
   if((k2>0.0?k2:-k2) < DBL_EPSILON){
      k2 = 0.0;
   }

   if(Precision < 0.0){
      Precision = -Precision;
   }
   if(Precision < DBL_EPSILON){
      Precision = DBL_EPSILON;
   }

   double Rd2 = Xd*Xd + Yd*Yd;
   double Rd  = sqrt(Rd2);


   if(Rd < DBL_EPSILON){
      Xu = 0.0;
      Yu = 0.0;
      return ERROR_SUCCESS;
   }

   double RuMax = CalculateRuMax(k1, k2);
   
   double k1x3 = 3.0*k1;
   double k2x5 = 5.0*k2;

   //Initialise Newton
   double Ru = Rd;
   if(Ru >= RuMax){
      Ru = RuMax/2.0;
   }

   double StepSize = DBL_MAX;

   int NewtonNotConverging = 0;
   int NewtonIterations = 0;

   
   while(StepSize > Precision){

      double Ru2   = Ru*Ru;
      double dF_Ru = 1 + Ru2*(k1x3 + k2x5*Ru2);

      double RuNew;

      if((dF_Ru>0.0?dF_Ru:-dF_Ru)>DBL_EPSILON){
         RuNew = Ru - (Ru*(1 + Ru2*(k1 + k2*Ru2)) - Rd)/dF_Ru;
      }else{
         RuNew = Ru - DBL_MAX;
      }

      StepSize = RuNew - Ru;
      if(StepSize < 0.0){
         StepSize = -StepSize;
      }
      Ru = RuNew;

      if((Ru < 0.0) || (Ru > RuMax)){
         NewtonNotConverging = 1;
         break;
      }

      NewtonIterations++;
      if(NewtonIterations >= 100){
         NewtonNotConverging = 1;
         break;
      }

   }

     if(NewtonNotConverging){

          double LeftLim  = 0.0;
      double RightLim = RuMax;
      Ru = RuMax/2.0;

      int BisectionIterations = 0;

      StepSize = DBL_MAX;
      while(StepSize > Precision){

         double Ru2  = Ru*Ru;
         double F_Ru = Ru*(1 + Ru2*(k1 + k2*Ru2)) - Rd;
         
         double RuNew;

         if(F_Ru > 0.0){
            RuNew = (Ru+LeftLim)/2.0;
            RightLim = Ru;
         }else if(F_Ru < 0.0){
            RuNew = (RightLim+Ru)/2.0;
            LeftLim = Ru;
         }else{
         
           break;
         }

         StepSize = RuNew - Ru;
         if(StepSize < 0.0){
            StepSize = -StepSize;
         }
         Ru = RuNew;

         BisectionIterations++;
      }
   }

   double DistortionFactor = Ru/Rd;
   Xu = Xd*DistortionFactor;
   Yu = Yd*DistortionFactor;
   
   return ERROR_SUCCESS;

}
double DistorsionConverter::CalculateRuMax(double k1, double k2){

   double RuMax = RU_INF;


   if(k2 != 0.0){


      double Radix = 9*k1*k1 - 20*k2;
      if((Radix>0.0?Radix:-Radix) < DBL_EPSILON){
         Radix = 0.0;
      }

      if( Radix >= 0.0 ){

         double k1x3N = -3*k1;
         double k2x10 = 10*k2;
         double RadixSqrt = sqrt(Radix);
         double Z1 = (k1x3N + RadixSqrt)/k2x10;
         double Z2 = (k1x3N - RadixSqrt)/k2x10;

         if(Z1 > 0.0){
            RuMax = sqrt(Z1);
         }
         if(Z2 > 0.0){
            double SqrtZ2 = sqrt(Z2);
            if(SqrtZ2 < RuMax){
               RuMax = SqrtZ2;
            }
         }
      }
   }else{

      if(k1 < 0.0){
         RuMax = sqrt(-1.0/(3*k1));
      }

   }
   return RuMax;

}


