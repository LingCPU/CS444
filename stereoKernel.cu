#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cuda_runtime.h>
#include <limits.h>

__global__ void stereoKernel(unsigned char* left, unsigned char* right,
                             double maxDisparity, int rows, int cols){

    // compute the row and col of the pixel to be processed
    int col = blockIdx.x*blockDim.x + threadIdx.x;
    int row = blockIdx.y*blockDim.y + threadIdx.y;

    // put your stereo matching code here
    // This code should only be for one pixel
    // See the video I posted on acceleration stereo on the GPU

    int disparityStep = 2;
    int windowStep = 2;
    const int windowWidth = 13;
    const int halfWindow = (windowWidth - 1) / 2;
    double contrastThreshold = 20;

    unsigned char leftPixel;
    unsigned char rightPixel;
    unsigned char centerPixel;
    unsigned char disp = (unsigned char)0;
    double sumDiff;
    double minSumDiff = (double)INT_MAX*(double)INT_MAX;
    double diff;
    double intensity, minIntensity, maxIntensity;

    if(row < halfWindow || row > rows - halfWindow || col < maxDisparity || col > cols - halfWindow){
        disparity[row * cols + col] = 0;
        return;
    }

    minIntensity = (double)(left[row * cols + col]);
    maxIntensity = minIntensity;

    for(int i = -halfWindow; i < halfWindow + 1; i += windowStep){
        for(int j = -halfWindow; j < halfWindow; j += windowStep){
            intensity = (double)(left[(row + i) * cols + (col + j)]);
            if(intensity < minIntensity) minIntensity = intensity;
            if(intensity > maxIntensity) maxIntensity = intensity;
        }
    }

    double contrast = maxIntensity - minIntensity;
    if(contrast < contrastThreshold){
        disparity[row * cols + col] = 0;
        return;
    }

    for(int k = 0; k < maxDisparity; k += disparityStep){
        sumDiff = 0.0;
        for(int i = -halfWindow; i < halfWindow + 1; i += windowStep){
            for(int j = -halfWindow; j < halfWindow + 1; j += windowStep){
                if(row + i < rows && col + j < cols && 0 <= col + j - k && col + j - k < cols){
                    leftPixel = left[(row + i) * cols + (col + j)];
                    rightPixel = right[(row + i) * cols + (col + j - k)];
                    diff = (double)leftPixel - (double)rightPixel;
                    sumDiff += fabs(diff);
                }
            }
        }
        if(sumDiff < minSumDiff){
            minSumDiff = sumDiff;
            disp = (unsigned char)l;
        }
    }

    disparity[row * cols + col] = disp;

}
