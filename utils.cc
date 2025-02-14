#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ctime>
#include <string>
#include <sstream>

using namespace std;

void vectorPrint(float* u , int rows){
  for(int i=0;i<rows;i++){
    printf( "%7.1f \n",u[i]);
  }
  printf("\n");

}

void vectorScale(float* u, int rows, float alpha , float* v){
  for(int i=0; i<rows; i++){
    v[i]=alpha*u[i];
  }
}

float vectorDotProduct(float* u, float* v, int rows){
  float result = 0.0;
  for(int i = 0; i < rows; i++){
    result += u[i] * v[i];
  }
  return result;
}

void vectorSubtract(float* u, float* v, int rows, float* w){
  for(int i = 0; i < rows; i++){
    w[i] = u[i] - v[i];
  }
}

float vectorNorm(float* u, int rows){
  return sqrt(vectorDotProduct(u, u, rows));
}

void buildMMatrix(float* m, int numPoints, float* x, float* y, float* z, int rows, int cols){
  int row = 0;
  for(int i = 0; i < numPoints; i++){
    // For the first row of each point:
    for(int j = 0; j < 4; j++){
      if(j == 0) m[row * cols + j] = x[i];
      if(j == 1) m[row * cols + j] = y[i];
      if(j == 2) m[row * cols + j] = z[i];
      if(j == 3) m[row * cols + j] = 1.0;
    }
    for(int j = 4; j < cols; j++){
      m[row * cols + j] = 0.0;
    }
    row++;

    // for the 2nd row of each point:
    for(int j = 0; j < 8; j++){
      m[row * cols + j] = 0.0;
    }
    for(int j = 4; j < 8; j++){
      if(j == 4) m[row*cols + j] = x[i];
      if(j == 5) m[row*cols + j] = y[i];
      if(j == 6) m[row*cols + j] = z[i];
      if(j == 7) m[row*cols + j] = 1.0;
    }
    for(int j = 4; j < cols; j++){
      m[row * cols + j] = 0.0;
    }
    row++;

    // for the 3rd row of each point:
    for(int j = 0; j < 8; j++){
      m[row * cols + j] = 0.0;
    }
  }
}