#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ctime>
#include <string>
#include <sstream>
#include "utils.h"

using namespace std;

int main(){
    float u[] = {1, 2, 3};
    float v[] = {4, 5, 6 };

    int rows = 3;
    float alpha = 2.0;

    vectorPrint(u, rows);

    vectorScale(u, rows, alpha, v);
    vectorPrint(v, rows);

    // HW 1 tests:
    float w[3]; // for results etc
    // use same rows/alpha from original test code
    
    // testing dotproduct
    float dot = vectorDotProduct(u, v, rows);
    printf("Dot product of u and v: %f\n\n", dot);

    // testing vector subtration
    printf("Vector v - u: \n");
    vectorSubtract(v, u, rows, w);
    vectorPrint(w, rows);

    // testing vector normalization
    float norm = vectorNorm(u, rows);
    printf("Normalization of vector u: %f\n\n", norm);







    return 0;
}
