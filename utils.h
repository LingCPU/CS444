#ifndef UTILS_H
#define UTILS_H


using namespace std;

void vectorPrint(float* u , int rows); 

void vectorScale(float* u, int rows, float alpha , float* v); 

float vectorDotProduct(float* u, float* v, int rows);

// void vectorScaling();

void vectorSubtract(float* u, float* v, int rows, float* w);

float vectorNorm(float* u, int rows);

#endif

