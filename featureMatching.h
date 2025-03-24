#ifndef FEATUREMATCHING_H
#define FEATUREMATCHING_H
 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
 
// Constants for the feature matching algorithm
#define MAX_FEATURES 500
#define MAX_DESC_SIZE 128
#define DEFAULT_SEARCH_WINDOW_WIDTH 64
#define DEFAULT_SEARCH_WINDOW_HEIGHT 5
 
// Struct for a 2D point
typedef struct{
    float x;
    float y;
} Point2D;
 
// Struct for a feature
typedef struct{
    Point2D position;           // Position of the feature in the image
    float descriptor[MAX_DESC_SIZE]; // Feature descriptor
    int descriptor_size;        // Actual size of the descriptor
} Feature;
 
// Structure for feature match
typedef struct{
    int left_idx;      // Index of the feature in the left image
    int right_idx;     // Index of the feature in the right image
    float similarity;  // Similarity score
    float disparity;   // Disparity (x_left - x_right)
} FeatureMatch;

// Structure for camera parameters
typedef struct{
    float focal_length;  // Focal length in pixels
    float baseline;      // Baseline between cameras in mm
    float cx_left;       // Principal point x-coordinate left camera
    float cy_left;       // Principal point y-coordinate left camera
    float cx_right;      // Principal point x-coordinate right camera
    float cy_right;      // Principal point y-coordinate right camera
} CameraParameters;

// Structure for disparity map
typedef struct{
    float* data;        // Disparity values
    uint8_t* confidence; // Confidence in the disparity value (0-255)
    int width;          // Width of the map
    int height;         // Height of the map
} DisparityMap;

// Initialize camera parameters with default values
void initCameraParameters(CameraParameters* params);


// Load camera parameters from calibration files
int loadCameraParameters(CameraParameters* params, const char* left_calib_file, const char* right_calib_file);


// Extract features from an image (not working shhhh)
int extractFeatures(const uint8_t* image, int width, int height, Feature* features, int max_features);

// Match features between left and right images using the algorithm from the image.
// For each feature in the left image, find the best match in the search region of the right image.
int matchFeatures(
    const Feature* left_features, int left_count,
    const Feature* right_features, int right_count,
    FeatureMatch* matches, int max_matches,
    int search_window_width, int search_window_height
);


// Create a disparity map from feature matches
int createDisparityMap(
    const FeatureMatch* matches, int match_count,
    const Feature* left_features,
    int width, int height,
    DisparityMap* disparity_map
);


// Compute depth from disparity
float computeDepth(float disparity, const CameraParameters* camera_params);


// Initialize a disparity map
int initDisparityMap(DisparityMap* map, int width, int height);


// Free memory allocated for disparity map
void freeDisparityMap(DisparityMap* map);


// Save disparity map to a file
int saveDisparityMap(const DisparityMap* map, const char* filename);


// Detect obstacles using the disparity map
int detectObstacles(
    const DisparityMap* disparity_map,
    const CameraParameters* camera_params,
    float min_depth, float max_depth,
    uint8_t* obstacle_map
);

#endif