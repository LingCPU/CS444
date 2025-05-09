#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "cuda_include.h"
#include <limits.h>

__global__ void stereoKernel(unsigned char* left, unsigned char* right, unsigned char* depth,
                             double maxDistance, int rows, int cols){

// compute the row and col of the pixel to be processed
int col = blockIdx.x*blockDim.x + threadIdx.x;
int row = blockIdx.y*blockDim.y + threadIdx.y;

// put your stereo matching code here
// This code should only be for one pixel
// See the video I posted on acceleration stereo on the GPU



// Check if we're within image bounds
    if(row >= rows || col >= cols || row < 0 || col < 0) return;
    
    // Parameters for stereo matching
    const int windowSize = 9;        // Window size for matching (must be odd)
    const int halfWindow = windowSize / 2;
    const int maxDisparity = 80;     // Maximum pixel displacement
    const int minDisparity = 4;      // Minimum pixel displacement to be consider valid
    
    const float baselineFocal = 60.0f * 560.0f; 
    if(col < halfWindow || col >= cols - halfWindow || row < halfWindow || row >= rows - halfWindow){
        depth[row * cols + col] = 0;
        return;
    }
    
    // Only process pixels with sufficient texture
    int textureThreshold = 8;
    int minVal = 0;
    int maxVal = 255;
    
    // Texture check
    for(int wy = -1; wy <= 1; wy++){
        for(int wx = -1; wx <= 1; wx++){
            int pixel = left[(row + wy) * cols + (col + wx)];
            if(pixel > maxVal) maxVal = pixel;
            if(pixel < minVal) minVal = pixel;
        }
    }
    
    if((maxVal - minVal) < textureThreshold){
        depth[row * cols + col] = 0;
        return;
    }
    
    // Find best match in right image
    int bestDisparity = 0;
    int minSAD = INT_MAX;  // Sum of Absolute Differences
    
    // Search from left to right in right image
    for(int d = 0; d < maxDisparity; d++){
        // Skip if we go outside the right image
        if(col - d < halfWindow) continue;
        int sad = 0;
        // Calculate sum of absolute differences for the window
        for(int wy = -halfWindow; wy <= halfWindow; wy+=2){
            for(int wx = -halfWindow; wx <= halfWindow; wx+=2){
                int leftPixel = left[(row + wy) * cols + (col + wx)];
                int rightPixel = right[(row + wy) * cols + (col + wx - d)];
                sad += abs(leftPixel - rightPixel);
            }
        }
        
        // Store the best match
        if(sad < minSAD){
            minSAD = sad;
            bestDisparity = d;
        }
    }
    
    int refinedMin = (bestDisparity > minDisparity) ? bestDisparity - 2 : minDisparity;
    int refinedMax = (bestDisparity < maxDisparity - 2) ? bestDisparity + 2 : maxDisparity;
    minSAD = INT_MAX;

    for(int d = refinedMin; d < refinedMax; d++) {
        if(col - d < halfWindow) continue;

        int sad = 0;
        for(int wy = -halfWindow; wy <= halfWindow; wy++) {
            for(int wx = -halfWindow; wx <= halfWindow; wx++) {
                int leftPixel = left[(row + wy) * cols + (col + wx)];
                int rightPixel = right[(row + wy) * cols + (col - d + wx)];
                sad += abs(leftPixel - rightPixel);
            }
        }

        if(sad < minSAD) {
            minSAD = sad;
            bestDisparity = d;
        }
    }

    float uniquenessRatio = 0.8f;
    for(int d = minDisparity; d < maxDisparity; d++) {
        if(d == bestDisparity) continue;

        if(col - d < halfWindow) continue;

        int sad = 0;
        for(int wy = -halfWindow; wy <= halfWindow; wy++) {
            for(int wx = -halfWindow; wx <= halfWindow; wx++) {
                int leftPixel = left[(row + wy) * cols + (col + wx)];
                int rightPixel = right[(row + wy) * cols + (col - d + wx)];
                sad += abs(leftPixel - rightPixel);
                // Early termination
                if(sad > minSAD / uniquenessRatio) break;
            }
            if(sad > minSAD / uniquenessRatio) break;
        }

        if(sad < minSAD / uniquenessRatio) {
            // Match is ambiguous, reject it
            depth[row * cols + col] = 0;
            return;
        }
    }
if(bestDisparity > minDisparity && bestDisparity < maxDisparity - 1) {
        int sadPrev = 0, sadNext = 0;

        // Calculate SAD for previous disparity
        for(int wy = -halfWindow; wy <= halfWindow; wy++) {
            for(int wx = -halfWindow; wx <= halfWindow; wx++) {
                int leftPixel = left[(row + wy) * cols + (col + wx)];
                int rightPixel = right[(row + wy) * cols + (col - (bestDisparity-1) + wx)];
                sadPrev += abs(leftPixel - rightPixel);
            }
        }

        // Calculate SAD for next disparity
        for(int wy = -halfWindow; wy <= halfWindow; wy++) {
            for(int wx = -halfWindow; wx <= halfWindow; wx++) {
                int leftPixel = left[(row + wy) * cols + (col + wx)];
                int rightPixel = right[(row + wy) * cols + (col - (bestDisparity+1) + wx)];
                sadNext += abs(leftPixel - rightPixel);
            }
        }

        // Sub-pixel correction using parabolic interpolation
        float delta = 0.5f * (sadPrev - sadNext) / (float)(sadPrev - 2*minSAD + sadNext);
        bestDisparity = bestDisparity + delta;
    }

    // Filter poor matches
    float threshold = windowSize * windowSize * 10;
    if(minSAD > threshold || bestDisparity < minDisparity){
        depth[row * cols + col] = 0;
        return;
    }
    
    // depth = baseline * focal_length / disparity
    float depthValue = baselineFocal / bestDisparity;
    
    // Scale depth for output image
    int depthScaled = (int)(255.0f * depthValue / maxDistance);
    
    // Clamp to valid range
    if(depthScaled > 255) depthScaled = 255;
    if(depthScaled < 0) depthScaled = 0;

    // Set the depth value in the output image
    depth[row * cols + col] = (unsigned char)depthScaled;
}
