#ifndef RECTIFICATION_H
#define RECTIFICATION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

void normalizeVector(float* v, int size);

void crossProduct(float* a, float* b, float* result);

void buildRrectMatrix(float* T, float* Rrect);

void rectifyPoint(float* R, float f, float* p, float* p_rectified);

void rectifyStereoSystem(float* R, float* T, float f, 
                        float** leftPoints, float** rightPoints, 
                        int numPoints, 
                        float** leftRectified, float** rightRectified);

float** allocate2DArray(int rows, int cols);

void free2DArray(float** array, int rows);

// Test function for the rectification algorithm.
void testRectificationAlgorithm();

#endif /* RECTIFICATION_H */