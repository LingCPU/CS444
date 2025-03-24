#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imageUtils.h"
#include "matrixUtils.h"
#include "utils.h"

int main(){

const int windowWidth = 11; //must be odd
const int halfWindow = (windowWidth-1)/2;
const int searchWidth = 71; //pixels must be odd
const char* leftBW  = "leftBW.ppm";
const char* rightBW = "warpedImg.ppm";
const char* depthImageName = "depth.ppm";
const char* disparityImageName = "disparity.ppm";
double pixelCorr[searchWidth];

PPMImage* leftImg; 
PPMImage* rightImg;

int cols     = 640;
int rows     = 480;
int maxColor = 255;
double baseLine = 60.0;
double focalLength = 560.0;
double maxDisparity = searchWidth;
double minDisparity = 50;
double maxDistance = baseLine*focalLength/minDisparity;
double distance;
double disparity;

//allocate memory for the output images
unsigned char* depthImage = (unsigned char*) malloc(rows*cols * sizeof(unsigned char));
unsigned char* disparityImage = (unsigned char*) malloc(rows*cols * sizeof(unsigned char));

//read images
leftImg   = readPPM(leftBW,0);
rightImg  = readPPM(rightBW,0);

// put your code here to do the stereo matching

return 0;
}
