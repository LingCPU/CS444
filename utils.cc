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

