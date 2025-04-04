#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cuda_runtime.h>
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
    const int maxDisparity = 64;     // Maximum pixel displacement
    const int minDisparity = 5;      // Minimum pixel displacement to be consider valid
    
    const float baselineFocal = 60.0f * 560.0f; 
    if(col < halfWindow || col >= cols - halfWindow || row < halfWindow || row >= rows - halfWindow){
        depth[row * cols + col] = 0;
        return;
    }
    
    // Only process pixels with sufficient texture
    int textureThreshold = 10;
    bool hasTexture = false;
    
    // Texture check
    for(int wy = -1; wy <= 1 && !hasTexture; wy++){
        for(int wx = -1; wx <= 1 && !hasTexture; wx++){
            int centerVal = left[(row) * cols + (col)];
            int neighborVal = left[(row + wy) * cols + (col + wx)];
            if(abs(centerVal - neighborVal) > textureThreshold) hasTexture = true;
        }
    }
    
    if(!hasTexture){
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
        for(int wy = -halfWindow; wy <= halfWindow; wy++){
            for(int wx = -halfWindow; wx <= halfWindow; wx++){
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