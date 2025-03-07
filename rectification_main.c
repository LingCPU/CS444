#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rectification.h"
#include "utils.h"
#include "matrixUtils.h"

int main(int argc, char *argv[]){
    printf("=== Stereo Image Rectification Test ===\n\n");
    testRectificationAlgorithm();
    printf("\n=== Test Complete ===\n");
    
    return 0;
}