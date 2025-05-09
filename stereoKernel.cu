#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "cuda_include.h"
#include <limits.h>

// Helper function to calculate absolute difference with saturation
__device__ inline unsigned char abs_diff(unsigned char a, unsigned char b) {
    return a > b ? a - b : b - a;
}

__global__ void stereoKernel(unsigned char* left, unsigned char* right, unsigned char* depth,
                             double maxDistance, int rows, int cols) {
    // Compute the row and col of the pixel to be processed
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;

    // Check if we're within image bounds
    if (row >= rows || col >= cols) return;
    
    // Parameters for stereo matching
    const int windowSize = 7;          // Smaller window for faster processing
    const int halfWindow = windowSize / 2;
    const int maxDisparity = 64;       // Adjusted based on expected scene depth
    const int minDisparity = 2;        // Lower minimum disparity to catch far objects
    
    // Baseline * focal length constant (in mm * pixels)
    const float baselineFocal = 60.0f * 560.0f;
    
    // Return early for border pixels
    if (col < halfWindow || col >= cols - halfWindow || 
        row < halfWindow || row >= rows - halfWindow) {
        depth[row * cols + col] = 0;
        return;
    }
    
    // Only process pixels with sufficient texture (increased threshold)
    const int textureThreshold = 10;   // Higher threshold for more robust matching
    unsigned char minVal = 255;
    unsigned char maxVal = 0;
    
    // Fast texture check with early termination
    for (int wy = -halfWindow; wy <= halfWindow && (maxVal - minVal) < textureThreshold; wy++) {
        for (int wx = -halfWindow; wx <= halfWindow && (maxVal - minVal) < textureThreshold; wx++) {
            unsigned char pixel = left[(row + wy) * cols + (col + wx)];
            minVal = min(minVal, pixel);
            maxVal = max(maxVal, pixel);
        }
    }
    
    if ((maxVal - minVal) < textureThreshold) {
        depth[row * cols + col] = 0;
        return;
    }
    
    // Find best match in right image
    int bestDisparity = 0;
    int minSAD = INT_MAX;          // Sum of Absolute Differences
    int secondBestSAD = INT_MAX;   // For uniqueness check
    
    // Search from left to right in right image (disparity space)
    for (int d = 0; d < maxDisparity; d++) {
        // Skip if we go outside the right image
        if (col - d < halfWindow) continue;
        
        int sad = 0;
        // Calculate sum of absolute differences for the window
        for (int wy = -halfWindow; wy <= halfWindow; wy++) {
            for (int wx = -halfWindow; wx <= halfWindow; wx++) {
                unsigned char leftPixel = left[(row + wy) * cols + (col + wx)];
                unsigned char rightPixel = right[(row + wy) * cols + (col + wx - d)];
                sad += abs_diff(leftPixel, rightPixel);
            }
        }
        
        // Store the best and second best match
        if (sad < minSAD) {
            secondBestSAD = minSAD;
            minSAD = sad;
            bestDisparity = d;
        } else if (sad < secondBestSAD) {
            secondBestSAD = sad;
        }
    }
    
    // Filter poor matches with dynamic threshold based on window size
    float sadThreshold = windowSize * windowSize * 8;  // Less restrictive threshold
    
    // Uniqueness ratio test (ensures the match is significantly better than second best)
    float uniquenessRatio = 0.8f;
    if (minSAD > sadThreshold || 
        bestDisparity < minDisparity || 
        secondBestSAD * uniquenessRatio < minSAD) {
        depth[row * cols + col] = 0;
        return;
    }
    
    // Sub-pixel refinement (parabolic interpolation)
    float subDisparity = bestDisparity;
    if (bestDisparity > 0 && bestDisparity < maxDisparity - 1) {
        // Get neighboring disparity costs
        int sadMinus = 0, sadPlus = 0;
        
        // Calculate SAD for bestDisparity-1
        for (int wy = -halfWindow; wy <= halfWindow; wy++) {
            for (int wx = -halfWindow; wx <= halfWindow; wx++) {
                unsigned char leftPixel = left[(row + wy) * cols + (col + wx)];
                unsigned char rightPixel = right[(row + wy) * cols + (col + wx - (bestDisparity - 1))];
                sadMinus += abs_diff(leftPixel, rightPixel);
            }
        }
        
        // Calculate SAD for bestDisparity+1
        for (int wy = -halfWindow; wy <= halfWindow; wy++) {
            for (int wx = -halfWindow; wx <= halfWindow; wx++) {
                unsigned char leftPixel = left[(row + wy) * cols + (col + wx)];
                unsigned char rightPixel = right[(row + wy) * cols + (col + wx - (bestDisparity + 1))];
                sadPlus += abs_diff(leftPixel, rightPixel);
            }
        }
        
        // Parabolic interpolation
        float denom = 2.0f * (sadMinus + sadPlus - 2 * minSAD);
        if (denom != 0) {
            float adjust = (sadMinus - sadPlus) / denom;
            if (adjust > -1 && adjust < 1) {
                subDisparity += adjust;
            }
        }
    }
    
    // Depth = baseline * focal_length / disparity
    float depthValue = baselineFocal / subDisparity;
    
    // Scale depth for output image with improved range mapping
    // Use a non-linear scaling to emphasize mid-range depths
    float normalizedDepth = depthValue / maxDistance;
    normalizedDepth = min(1.0f, normalizedDepth);  // Clamp to [0,1]
    
    // Apply gamma correction for better visualization
    float gamma = 0.5f;  // Adjust as needed (< 1 brightens, > 1 darkens)
    float correctedDepth = powf(normalizedDepth, gamma);
    
    int depthScaled = (int)(255.0f * correctedDepth);
    
    // Ensure valid range
    depthScaled = max(0, min(255, depthScaled));
    
    // Set the depth value in the output image
    depth[row * cols + col] = (unsigned char)depthScaled;
}