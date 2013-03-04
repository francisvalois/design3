#include "Matrix.h"


using namespace  std;
Matrix::Matrix(int _nb_row, int _nb_col){

   NbRow = 0;
   NbCol = 0;

   el = 0;

   SetSize(_nb_row, _nb_col);

}

Matrix::Matrix(const Matrix &_m){

   NbRow = 0;
   NbCol = 0;

   el = 0;

   Equal(_m);

}


Matrix::~Matrix(){

   if(el){
      for(int i=0; i<NbRow; i++){
         if(el[i]) delete [] el[i];
      }
      delete [] el;
   }

}

double Matrix::Det() const{

   if(NbRow != NbCol) return 0.0;
   if(NbRow == 0)     return 0.0;

   Matrix tmp = *this;
   double det = tmp.RowReduceToEchelonForm();
   for(int i=0; i<NbCol; i++){
      det *= tmp.el[i][i];
   }

   return det;
}

void Matrix::Equal(const Matrix &_m){

   SetSize(_m.NbRow, _m.NbCol);

   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){
         el[i][j] = _m.el[i][j];
      }
   }

}


void Matrix::I(){

   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){
         if(i!=j) el[i][j] = 0;
         else el[i][j] = 1;
      }
   }
}

//Return the determinant. If not invertible, det == 0.
double Matrix::Invert(){

   if(NbRow != NbCol) return 0.0;
   if(NbRow == 0)     return 0.0;

   Matrix Inv = *this;
   Matrix Identity(NbRow, NbCol);
   Identity.I();
   Inv |= Identity;

   double det = Inv.RowReduceToReducedEchelonForm();

   if(det != 0.0){

      for(int i=0; i<NbRow; i++){
         for(int j=0; j<NbCol; j++){
            el[i][j] = Inv.el[i][j+NbCol];
         }
      }

   }else{

      SetSize(0,0);

   }

   return det;

}

double Matrix::RowReduceToEchelonForm(){

   if( (NbRow == 0)||(NbCol == 0) ) return 0.0;

   int MaxNbOfPivots = NbRow < NbCol ? NbRow : NbCol;
   int LastPivCol = -1;

   int    NbRowInterchanges = 0;
   double DetFactor         = 1.0;
   if(NbRow > MaxNbOfPivots) DetFactor = 0.0;

   for(int Row = 0; Row < MaxNbOfPivots; Row++){
   
      //Find a pivot column (the leftmost nonzero column)
      int PivCol = -1;
      for(int Col=LastPivCol+1; Col<NbCol; Col++){

         //Take the max element of this column
         int RowMax = Row;
         double RowMaxEl = el[RowMax][Col];
         for(int i=RowMax+1; i<NbRow; i++){

            if(el[i][Col] > RowMaxEl){
               RowMax = i;
               RowMaxEl = el[RowMax][Col];
            }
         }
   
         if( ((RowMaxEl>0.0)?RowMaxEl:-RowMaxEl) > MATRIX_ZERO_LIM ){

            if(RowMax != Row){

               double *tmp = el[Row];
               el[Row] = el[RowMax];
               el[RowMax] = tmp;
               NbRowInterchanges++;

            }
            PivCol = Col;
            break;
         }
      }
      if(PivCol != -1){

         double PivVal = el[Row][PivCol];
         for(int i=Row+1; i<NbRow; i++){

            double PivRowScale = el[i][PivCol]/PivVal;
            for(int j=PivCol; j<NbCol; j++){
               double OldVal = el[i][j];
               double NewVal = el[i][j] -= el[Row][j]*PivRowScale;
               OldVal = (OldVal>0?OldVal:-OldVal);
               NewVal = (NewVal>0?NewVal:-NewVal);
               if(NewVal < (OldVal/MATRIX_NUM_PRECISION_DOUBLE) ) el[i][j] = 0.0;
            }

         }

         LastPivCol = PivCol;
         if(PivCol >= MaxNbOfPivots) DetFactor = 0.0;

      }else{

         DetFactor = 0.0;
         break;

      }

   
   }

  
   if(NbRowInterchanges%2) return -1.0*DetFactor;
   else                    return  DetFactor;

}

double Matrix::RowReduceToReducedEchelonForm(){

   if( (NbRow == 0)||(NbCol == 0) ) return 0.0;

   double DetFactor = RowReduceToEchelonForm();

   for(int Row = (NbRow-1); Row >= 0; Row--){

      for(int Col = 0; Col < NbCol; Col++){

         if( (el[Row][Col]>0.0?el[Row][Col]:-el[Row][Col]) > MATRIX_ZERO_LIM){

            double PivRowScale = 1.0/el[Row][Col];
            DetFactor *= el[Row][Col];
            for(int j=Col; j<NbCol; j++){
               el[Row][j] *= PivRowScale;
            }
  
            for(int i=0; i<Row; i++){
               double Scale = el[i][Col];
               for(int j=Col; j<NbCol; j++){
                  double OldVal = el[i][j];
                  double NewVal = el[i][j] -= el[Row][j]*Scale;
                  OldVal = (OldVal>0?OldVal:-OldVal);
                  NewVal = (NewVal>0?NewVal:-NewVal);
                  if(NewVal < (OldVal/MATRIX_NUM_PRECISION_DOUBLE) ) el[i][j] = 0.0;
               }
            }
            break;
         }

      }
   }
   
   return DetFactor;

}

int Matrix::SetSize(int _nb_row, int _nb_col){

   if( (NbRow == _nb_row) && (NbCol == _nb_col) ) return 0;

   if(_nb_row < 0) _nb_row = 0;
   if(_nb_col < 0) _nb_col = 0;

   if(el){

      for(int i=0; i<NbRow; i++){
         if(el[i]){
            delete [] el[i];
         }
      }

      delete [] el;
      el = 0;
   }

   NbRow = _nb_row;
   NbCol = _nb_col;

   if( (NbRow!=0) && (NbCol!=0) ){

      el = new double*[NbRow];
      for(int i=0; i<NbRow; i++) el[i] = new double[NbCol];

      for(int j=0; j<NbRow; j++){
         for(int k=0; k<NbCol; k++) el[j][k] = 0;
      }
            

   }else{

      NbRow = NbCol = 0;

   }

   return 0;

}

void Matrix::Transpose(){

   if( (NbRow == 0)||(NbCol == 0) ) return;

   Matrix Trans(NbCol, NbRow);

   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){

         Trans.el[j][i] = el[i][j];
      
      }
   }

   *this = Trans;
}

int Matrix::operator==(const Matrix &_m) const{

   if( (NbRow != _m.NbRow)||(NbCol != _m.NbCol) ) return 0;

   int Equal = 1; 

   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){
         if(el[i][j] != _m.el[i][j]){
            Equal = 0;
         }
      }
   }

   return Equal;

}

const Matrix &Matrix::operator=(const Matrix &_m){

   Equal(_m);

   return *this;

}

Matrix Matrix::operator+(const Matrix &_m) const{
   if( (NbRow == _m.NbRow)&&(NbCol == _m.NbCol) ){

      Matrix tmp(*this);
      for(int i=0; i<tmp.NbRow; i++){
         for(int j=0; j<tmp.NbCol; j++){
            tmp.el[i][j] += _m.el[i][j];
         }
      }
      return tmp;

   }else{

      Matrix EmptyMatrix;
      return EmptyMatrix;

   }

}


const Matrix &Matrix::operator+=(const Matrix &_m){

   if( (NbRow == _m.NbRow)&&(NbCol == _m.NbCol) ){

      for(int i=0; i<NbRow; i++){
         for(int j=0; j<NbCol; j++){
            el[i][j] += _m.el[i][j];
         }
      }

   }else{

      SetSize(0,0);
      
   }

   return *this;

}


Matrix Matrix::operator-(const Matrix &_m) const{

     if( (NbRow == _m.NbRow)&&(NbCol == _m.NbCol) ){

      Matrix tmp(*this);
      for(int i=0; i<tmp.NbRow; i++){
         for(int j=0; j<tmp.NbCol; j++){
            tmp.el[i][j] -= _m.el[i][j];
         }
      }
      return tmp;

   }else{

      Matrix EmptyMatrix;
      return EmptyMatrix;

   }

}

const Matrix &Matrix::operator-=(const Matrix &_m){

   if( (NbRow == _m.NbRow)&&(NbCol == _m.NbCol) ){

      for(int i=0; i<NbRow; i++){
         for(int j=0; j<NbCol; j++){
            el[i][j] -= _m.el[i][j];
         }
      }

   }else{

      SetSize(0,0);
      
   }

   return *this;

}

Matrix Matrix::operator*(const double &_s) const{

   Matrix Res = *this;
   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){
         Res.el[i][j] *= _s;
      }
   }
   return Res;

}

Matrix Matrix::operator*(const Matrix &_m2) const{

   const Matrix &_m1 = *this;
   if(_m1.NbCol != _m2.NbRow){

      Matrix EmptyMatrix;
      return EmptyMatrix;

   }else{

      Matrix Res(_m1.NbRow, _m2.NbCol);
      for(int i=0; i<_m1.NbRow; i++){
         for(int j=0; j<_m2.NbCol; j++){
            Res.el[i][j] = 0;
            for(int k=0; k<_m1.NbCol; k++){
               Res.el[i][j] += _m1.el[i][k]*_m2.el[k][j];
            }
         }
      }
      return Res;

   }

}

const Matrix &Matrix::operator*=(const double &_s){

   for(int i=0; i<NbRow; i++){
      for(int j=0; j<NbCol; j++){
         el[i][j] *= _s;
      }
   }
   return *this;

}

const Matrix &Matrix::operator*=(const Matrix &_m2){


   if(NbCol != _m2.NbRow){

      SetSize(0,0);

   }else{

      const Matrix _m1 = *this;
      SetSize(_m1.NbRow, _m2.NbCol);
      for(int i=0; i<_m1.NbRow; i++){
         for(int j=0; j<_m2.NbCol; j++){
            el[i][j] = 0;
            for(int k=0; k<_m1.NbCol; k++){
               el[i][j] += _m1.el[i][k]*_m2.el[k][j];
            }
         }
      }

   }
   return *this;

}

Matrix Matrix::operator|(const Matrix &_m2) const{

   const Matrix &_m1 = *this;

   if(_m1.NbRow != _m2.NbRow){

      Matrix EmptyMatrix;
      return EmptyMatrix;

   }else{

      int ResNbCol = _m1.NbCol+_m2.NbCol;
      Matrix Res(_m1.NbRow, ResNbCol);
      for(int i=0; i<_m1.NbRow; i++){
         for(int j=0; j<_m1.NbCol; j++){
            Res.el[i][j] = _m1.el[i][j];
         }
         for(int k=0; k<_m2.NbCol; k++){
            Res.el[i][k+_m1.NbCol] = _m2.el[i][k];
         }
      }
      return Res;

   }
}

const Matrix &Matrix::operator|=(const Matrix &_m2){

   if(NbRow != _m2.NbRow){

      SetSize(0,0);

   }else{

      Matrix _m1 = *this;
      int ResNbCol = _m1.NbCol+_m2.NbCol;
      SetSize(_m1.NbRow, ResNbCol);
      for(int i=0; i<_m1.NbRow; i++){
         for(int j=0; j<_m1.NbCol; j++){
            el[i][j] = _m1.el[i][j];
         }
         for(int k=0; k<_m2.NbCol; k++){
            el[i][k+_m1.NbCol] = _m2.el[i][k];
         }
      }

   }
   
   return *this;

}

#include <iostream>
void Matrix::Print(){

   if( (NbRow == 0)||(NbCol == 0) ){

	  cout << "Matrice vide !" << endl;

   }else{

      for(int i=0; i<NbRow; i++){

         for(int j=0; j<NbCol; j++){

            cout << el[i][j] << " ";

         }
         cout << endl;

      }
      cout << endl;

   }

}








