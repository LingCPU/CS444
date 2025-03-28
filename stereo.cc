#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "imageUtils.h"
#include "matrixUtils.h"
#include "utils.h"

typedef struct{
    int x, y;               // Position in image
    unsigned char desc[25]; // Simple descriptor (5x5 patch)
} Feature;

typedef struct{
    int left_idx;           // Index in left image features
    int right_idx;          // Index in right image features
    double similarity;      // How similar the features are
    double disparity;       // Horizontal displacement
} Match;

// Check if a pixel is a corner by looking at intensity differences
int isCorner(const unsigned char* image, int width, int height, int x, int y, int threshold){
    // Don't check corners near image borders
    if(x < 3 || x >= width - 3 || y < 3 || y >= height - 3) return 0;

    unsigned char center = image[y * width + x];
    int brighter = 0;
    int darker = 0;

    // Check pixels in a circle around the point
    for(int i = -3; i <= 3; i++){
        for(int j = -3; j <= 3; j++){
            // Skip center and corners of the square
            if((i == 0 && j == 0) || (abs(i) == 3 && abs(j) == 3)) continue;
            unsigned char pixel = image[(y + i) * width + (x + j)];
            if(pixel > center + threshold) brighter++;
            else if(pixel < center - threshold) darker++;
        }
    }
    // It's a corner if there are enough brighter or darker pixels
    return (brighter >= 8 || darker >= 8);
}

// Create a simple descriptor by sampling a 5x5 patch around the feature
void createDescriptor(const unsigned char* image, int width, int height, int x, int y, unsigned char* descriptor){
    int idx = 0;
    // Sample a 5x5 grid around the feature point
    for(int i = -2; i <= 2; i++){
        for(int j = -2; j <= 2; j++){
            int px = x + j;
            int py = y + i;
            
            // Check bounds
            if(px < 0) px = 0;
            if(px >= width) px = width - 1;
            if(py < 0) py = 0;
            if(py >= height) py = height - 1;
            descriptor[idx++] = image[py * width + px];
        }
    }
}

// Calculate similarity between two descriptors (lower is more similar)
double calculateSimilarity(const unsigned char* desc1, const unsigned char* desc2, int size){
    double sum = 0;
    for(int i = 0; i < size; i++){
        double diff = (double)desc1[i] - (double)desc2[i];
        sum += diff * diff;
    }
    return sum;
}

// Detect features in an image
int detectFeatures(const unsigned char* image, int width, int height, Feature* features, int max_features){
    int threshold = 20;  // Intensity difference threshold
    int count = 0;
    
    // Scan the image looking for corners
    for(int y = 3; y < height - 3; y++){
        for(int x = 3; x < width - 3; x++){
            if(isCorner(image, width, height, x, y, threshold)){
                // We found a corner, add it to our features
                if(count < max_features){
                    features[count].x = x;
                    features[count].y = y;
                    createDescriptor(image, width, height, x, y, features[count].desc);
                    count++;
                }
            }
        }
    }
    return count;
}

// Match features between left and right images
int matchFeatures(const Feature* left_features, int left_count, const Feature* right_features, int right_count, Match* matches, int max_matches, int max_disparity){
    int match_count = 0;
    
    // For each feature in left image
    for(int i = 0; i < left_count && match_count < max_matches; i++){
        double best_similarity = 1e9;
        int best_match = -1;
        
        // Look for matches in right image
        for(int j = 0; j < right_count; j++){
            // Check if features are at similar y-coordinates
            if(abs(left_features[i].y - right_features[j].y) > 2) continue;
                
            // Check if disparity is in valid range (right feature should be to the left)
            int disparity = left_features[i].x - right_features[j].x;
            if(disparity < 0 || disparity > max_disparity) continue;
                
            // Calculate similarity between descriptors
            double similarity = calculateSimilarity(left_features[i].desc, right_features[j].desc, 25);  // 25 is 5x5 descriptor
            
            // Update best match if this one is better
            if(similarity < best_similarity){
                best_similarity = similarity;
                best_match = j;
            }
        }
        // If we found a good match, add it
        if(best_match >= 0 && best_similarity < 2000){  // Threshold for good match
            matches[match_count].left_idx = i;
            matches[match_count].right_idx = best_match;
            matches[match_count].similarity = best_similarity;
            matches[match_count].disparity = left_features[i].x - right_features[best_match].x;
            match_count++;
        }
    }
    return match_count;
}

int main(){
    const int windowWidth = 11; // must be odd
    const int halfWindow = (windowWidth-1)/2;
    const int searchWidth = 71; // pixels must be odd
    const char* leftBW  = "leftBW.ppm";
    const char* rightBW = "warpedImg.ppm";
    const char* depthImageName = "depth.ppm";
    const char* disparityImageName = "disparity.ppm";
    double pixelCorr[searchWidth];

    PPMImage* leftImg; 
    PPMImage* rightImg;

    int cols = 640;
    int rows = 480;
    int maxColor = 255;
    double baseLine = 60.0;
    double focalLength = 560.0;
    double maxDisparity = searchWidth;
    double minDisparity = 50;
    double maxDistance = baseLine*focalLength/minDisparity;
    double distance;
    double disparity;

    //allocate memory for the output images
    unsigned char* depthImage = (unsigned char*) malloc(rows * cols * sizeof(unsigned char));
    unsigned char* disparityImage = (unsigned char*) malloc(rows * cols * sizeof(unsigned char));

    //read images
    leftImg = readPPM(leftBW,0);
    rightImg = readPPM(rightBW,0);
    
    // Initialize output images to black
    memset(depthImage, 0, rows*cols * sizeof(unsigned char));
    memset(disparityImage, 0, rows*cols * sizeof(unsigned char));
    
    // Max number of features and matches
    const int MAX_FEATURES = 1000;
    const int MAX_MATCHES = 1000;
    
    // Allocate memory for features and matches
    Feature* left_features = (Feature*) malloc(MAX_FEATURES * sizeof(Feature));
    Feature* right_features = (Feature*) malloc(MAX_FEATURES * sizeof(Feature));
    Match* matches = (Match*) malloc(MAX_MATCHES * sizeof(Match));
    
    if(!left_features || !right_features || !matches){
        printf("Failed to allocate memory\n");
        return -1;
    }
    
    // Detect features in both images
    printf("Detecting features in left image...\n");
    int left_count = detectFeatures(leftImg->data, cols, rows, left_features, MAX_FEATURES);
    printf("Found %d features in left image\n", left_count);
    
    printf("Detecting features in right image...\n");
    int right_count = detectFeatures(rightImg->data, cols, rows, right_features, MAX_FEATURES);
    printf("Found %d features in right image\n", right_count);
    
    // Match features between images
    printf("Matching features...\n");
    int match_count = matchFeatures(left_features, left_count, right_features, right_count, matches, MAX_MATCHES, searchWidth);
    printf("Found %d matches\n", match_count);
    
    // Create disparity map from matches
    printf("Creating disparity map...\n");
    for(int i = 0; i < match_count; i++){
        int x = left_features[matches[i].left_idx].x;
        int y = left_features[matches[i].left_idx].y;
        disparity = matches[i].disparity;
        
        // Only process valid disparities
        if(disparity >= minDisparity && disparity <= maxDisparity){
            // Calculate depth from disparity
            distance = baseLine * focalLength / disparity;
            
            // Paint a small square around the feature to make it more visible
            for(int dx = -2; dx <= 2; dx++){
                for(int dy = -2; dy <= 2; dy++){
                    int nx = x + dx;
                    int ny = y + dy;
                    if(nx >= 0 && nx < cols && ny >= 0 && ny < rows){
                        disparityImage[ny * cols + nx] = (unsigned char)(255 * disparity / maxDisparity);
                        depthImage[ny * cols + nx] = (unsigned char)(255 * distance / maxDistance);
                    }
                }
            }
        }
    }
    
    // Create a smoother disparity map by propagating values
    printf("Smoothing disparity map...\n");
    for(int y = halfWindow; y < rows - halfWindow; y++){
        for (int x = halfWindow; x < cols - halfWindow; x++){
            // Skip if already processed
            if(disparityImage[y * cols + x] > 0) continue;
                
            // Look for nearby features with valid disparity
            int count = 0;
            double total_disparity = 0;
            
            for(int dy = -halfWindow; dy <= halfWindow; dy++){
                for(int dx = -halfWindow; dx <= halfWindow; dx++){
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    if(disparityImage[ny * cols + nx] > 0){
                        total_disparity += disparityImage[ny * cols + nx];
                        count++;
                    }
                }
            }
            
            // If we found enough neighbors, interpolate
            if(count > 3){
                double avg_disparity = total_disparity / count;
                disparityImage[y * cols + x] = (unsigned char)avg_disparity;
                
                // Calculate depth from disparity
                if(avg_disparity > 0){
                    distance = baseLine * focalLength / (avg_disparity * maxDisparity / 255.0);
                    depthImage[y * cols + x] = (unsigned char)(255 * distance / maxDistance);
                }
            }
        }
    }
    
    // Free allocated memory
    free(left_features);
    free(right_features);
    free(matches);
    
    // write the disparity and depth images to files
    writePPM(depthImageName, cols, rows, maxColor, 0, depthImage);
    writePPM(disparityImageName, cols, rows, maxColor, 0, disparityImage);

    // Clean up
    freePPM(leftImg);
    freePPM(rightImg);
    free(depthImage);
    free(disparityImage);

    return 0;
}