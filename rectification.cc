#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "rectification.h"
#include "utils.h"
#include "matrixUtils.h"

void normalizeVector(float* v, int size) {
    float norm = 0.0;
    for (int i = 0; i < size; i++) {
        norm += v[i] * v[i];
    }
    norm = sqrt(norm);
    
    if (norm > 1e-10) { // Avoid division by zero
        for (int i = 0; i < size; i++) {
            v[i] /= norm;
        }
    }
}

void crossProduct(float* a, float* b, float* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void buildRrectMatrix(float* T, float* Rrect) {
    float e1[3], e2[3], e3[3];
    float opticalAxis[3] = {0, 0, 1}; // Direction of optical axis
    
    // e1 = T / ||T|| (normalized translation vector)
    memcpy(e1, T, 3 * sizeof(float));
    normalizeVector(e1, 3);
    
    // e2 = 1/sqrt(Tx^2 + Ty^2) * [-Ty, Tx, 0]
    float denominator = sqrt(T[0]*T[0] + T[1]*T[1]);
    if (denominator > 1e-10) { // Avoid division by zero
        e2[0] = -T[1] / denominator;
        e2[1] = T[0] / denominator;
        e2[2] = 0.0;
    } else {
        // Handle the case where T is along Z-axis
        e2[0] = 1.0;
        e2[1] = 0.0;
        e2[2] = 0.0;
    }
    
    // e3 = e1 × e2 (cross product)
    crossProduct(e1, e2, e3);
    
    // Build the Rrect matrix as [e1^T; e2^T; e3^T]
    for (int i = 0; i < 3; i++) {
        Rrect[i*3 + 0] = e1[i];
        Rrect[i*3 + 1] = e2[i];
        Rrect[i*3 + 2] = e3[i];
    }
    
    // Transpose to get the correct rotation matrix
    float RrectTemp[9];
    memcpy(RrectTemp, Rrect, 9 * sizeof(float));
    matrixTranspose(RrectTemp, 3, 3, Rrect);
}

void rectifyPoint(float* R, float f, float* p, float* p_rectified) {
    float rotated[3];
    
    // Compute R*p
    matrixTimesVector(R, 3, 3, p, 3, rotated);
    
    // Scale by f/z'
    float scale = f / rotated[2];
    p_rectified[0] = rotated[0] * scale;
    p_rectified[1] = rotated[1] * scale;
    p_rectified[2] = rotated[2] * scale;
}

void rectifyStereoSystem(float* R, float* T, float f, 
                        float** leftPoints, float** rightPoints, 
                        int numPoints, 
                        float** leftRectified, float** rightRectified) {
    // Step 1: Build the Rrect matrix
    float Rrect[9];
    buildRrectMatrix(T, Rrect);
    
    printf("Rectification matrix Rrect:\n");
    matrixPrint(Rrect, 3, 3);
    
    // Step 2: Set Rl = Rrect and Rr = R*Rrect
    float Rl[9], Rr[9];
    memcpy(Rl, Rrect, 9 * sizeof(float));
    
    matrixProduct(R, 3, 3, Rrect, 3, 3, Rr);
    
    printf("Left camera rotation Rl:\n");
    matrixPrint(Rl, 3, 3);
    printf("Right camera rotation Rr:\n");
    matrixPrint(Rr, 3, 3);
    
    // Steps 3 & 4: Rectify each point in both cameras
    for (int i = 0; i < numPoints; i++) {
        rectifyPoint(Rl, f, leftPoints[i], leftRectified[i]);
        rectifyPoint(Rr, f, rightPoints[i], rightRectified[i]);
    }
}

float** allocate2DArray(int rows, int cols) {
    float** array = (float**)malloc(rows * sizeof(float*));
    for (int i = 0; i < rows; i++) {
        array[i] = (float*)malloc(cols * sizeof(float));
    }
    return array;
}

void free2DArray(float** array, int rows) {
    for (int i = 0; i < rows; i++) {
        free(array[i]);
    }
    free(array);
}

void testRectificationAlgorithm() {
    // Example parameters
    float f = 500.0; // Focal length in pixels
    
    // Rotation between left and right cameras - small rotation for testing
    float R[9] = {   
        0.9998, 0.0175, -0.0087,
        -0.0175, 0.9998, -0.0087,
        0.0087, 0.0087, 0.9999
    };
    
    // Translation (baseline along X-axis, 100 pixels)
    float T[3] = {-100.0, 0.0, 0.0}; 
    
    // Example points (3D homogeneous coordinates)
    int numPoints = 5;
    float** leftPoints = allocate2DArray(numPoints, 3);
    float** rightPoints = allocate2DArray(numPoints, 3);
    float** leftRectified = allocate2DArray(numPoints, 3);
    float** rightRectified = allocate2DArray(numPoints, 3);
    
    // Initialize some example points
    for (int i = 0; i < numPoints; i++) {
        leftPoints[i][0] = 100.0 + i * 50.0;  // x
        leftPoints[i][1] = 100.0 + i * 20.0;  // y
        leftPoints[i][2] = f;                 // z = f for normalized coordinates
        
        rightPoints[i][0] = 80.0 + i * 50.0;  // x (slightly shifted for parallax)
        rightPoints[i][1] = 105.0 + i * 20.0; // y
        rightPoints[i][2] = f;                // z = f
    }
    
    // Apply rectification
    rectifyStereoSystem(R, T, f, leftPoints, rightPoints, numPoints, leftRectified, rightRectified);
    
    // Print results
    printf("\nOriginal and rectified points:\n");
    printf("%-15s %-15s %-15s %-15s %-15s %-15s\n", 
           "Left X", "Left Y", "Right X", "Right Y", "Rect Left Y", "Rect Right Y");
    
    for (int i = 0; i < numPoints; i++) {
        printf("%-15.2f %-15.2f %-15.2f %-15.2f %-15.2f %-15.2f\n",
               leftPoints[i][0], leftPoints[i][1],
               rightPoints[i][0], rightPoints[i][1],
               leftRectified[i][1], rightRectified[i][1]);
    }
    
    // Check if y-coordinates are aligned (should be the same in rectified images)
    printf("\nVerification - differences in rectified y-coordinates:\n");
    for (int i = 0; i < numPoints; i++) {
        float diff = leftRectified[i][1] - rightRectified[i][1];
        printf("Point %d: diff = %.6f\n", i, diff);
    }
    
    // Free allocated memory
    free2DArray(leftPoints, numPoints);
    free2DArray(rightPoints, numPoints);
    free2DArray(leftRectified, numPoints);
    free2DArray(rightRectified, numPoints);
}