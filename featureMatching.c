#include "featureMatching.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

// Initialize camera parameters with default values
void initCameraParameters(CameraParameters* params){
    if(params == NULL) return;
    params->focal_length = 560.0f; 
    params->baseline = 65.0f;       // Distance between cameras in mm
    params->cx_left = 350.0f;       // Principal point x-coordinate (left camera)
    params->cy_left = 224.0f;       // Principal point y-coordinate (left camera)
    params->cx_right = 361.0f;      // Principal point x-coordinate (right camera)
    params->cy_right = 220.0f;      // Principal point y-coordinate (right camera)
}

// Load camera parameters from calibration files
int loadCameraParameters(CameraParameters* params, const char* left_calib_file, const char* right_calib_file){
    FILE *left_file, *right_file;
    int width, height, num_points;
    
    // Initialize with default values
    initCameraParameters(params);

    // Open left calibration file
    left_file = fopen(left_calib_file, "r");
    if(left_file == NULL){
        printf("Error: Cannot open left calibration file %s\n", left_calib_file);
        return 0;
    }
    // Read image dimensions and number of calibration points
    fscanf(left_file, "%d %d %d", &width, &height, &num_points);
    fclose(left_file);
    
    // Open right calibration file
    right_file = fopen(right_calib_file, "r");
    if(right_file == NULL){
        printf("Error: Cannot open right calibration file %s\n", right_calib_file);
        return 0;
    }
    // Read image dimensions and number of calibration points
    fscanf(right_file, "%d %d %d", &width, &height, &num_points);
    fclose(right_file);
    return 1;
}

// Calculate similarity between two descriptors (using L2 distance)
static float calculateSimilarity(const float* desc1, const float* desc2, int size){
    float sum_squared_diff = 0.0f;
    for(int i = 0; i < size; i++){
        float diff = desc1[i] - desc2[i];
        sum_squared_diff += diff * diff;
    }
    // Return the inverse of the distance, so higher values mean more similar
    if(sum_squared_diff < 1e-10f) return FLT_MAX;
    return 1.0f / sqrtf(sum_squared_diff);
}

// Match features between left and right images
int matchFeatures(const Feature* left_features, int left_count, const Feature* right_features, int right_count, FeatureMatch* matches, int max_matches, int search_window_width, int search_window_height){
    int match_count = 0;
    // For each feature in the left image
    for (int i = 0; i < left_count && match_count < max_matches; i++) {
        const Feature* left_feature = &left_features[i];
        float best_similarity = 0.0f;
        int best_match_idx = -1;
        
        // Define search region
        float y_min = left_feature->position.y - search_window_height / 2.0f;
        float y_max = left_feature->position.y + search_window_height / 2.0f;
        
        // Features in the right image should be to the left of the left image feature because of camera orientation
        float x_min = 0;
        float x_max = left_feature->position.x;

        for (int j = 0; j < right_count; j++) {
            const Feature* right_feature = &right_features[j];
            // Check if the right feature is in the search region
            if(right_feature->position.y >= y_min && right_feature->position.y <= y_max && right_feature->position.x >= x_min && right_feature->position.x <= x_max){            
                float similarity = calculateSimilarity(
                    left_feature->descriptor, 
                    right_feature->descriptor, 
                    left_feature->descriptor_size
                );
                if(similarity > best_similarity){
                    best_similarity = similarity;
                    best_match_idx = j;
                }
            }
        }
        // If we found a match
        if(best_match_idx >= 0){
            matches[match_count].left_idx = i;
            matches[match_count].right_idx = best_match_idx;
            matches[match_count].similarity = best_similarity;
            float disparity = left_feature->position.x - right_features[best_match_idx].position.x;
            matches[match_count].disparity = disparity;
            match_count++;
        }
    }
    return match_count;
}

// Create a disparity map from feature matches
int createDisparityMap(const FeatureMatch* matches, int match_count, const Feature* left_features, int width, int height, DisparityMap* disparity_map){
    // Initialize the disparity map
    if(!initDisparityMap(disparity_map, width, height)) return 0;
    // Clear the disparity map
    memset(disparity_map->data, 0, width * height * sizeof(float));
    memset(disparity_map->confidence, 0, width * height * sizeof(uint8_t));
    // Place disparities at feature points
    for(int i = 0; i < match_count; i++){
        int left_idx = matches[i].left_idx;
        float disparity = matches[i].disparity;
        
        // Get the coordinates of the feature in the left image
        int x = (int)left_features[left_idx].position.x;
        int y = (int)left_features[left_idx].position.y;
        
        // Check if the coordinates are within the image
        if(x >= 0 && x < width && y >= 0 && y < height){
            disparity_map->data[y * width + x] = disparity;
            disparity_map->confidence[y * width + x] = 255;
        }
    }
    // Perform propagation to create a dense map
    const int kernel_size = 7;
    const int half_kernel = kernel_size / 2;
    // Create a temporary buffer for the propagated disparity
    float* temp_disparity = (float*)malloc(width * height * sizeof(float));
    uint8_t* temp_confidence = (uint8_t*)malloc(width * height * sizeof(uint8_t));
    if(temp_disparity == NULL || temp_confidence == NULL) {
        free(temp_disparity);
        free(temp_confidence);
        return 0;
    }
    // Copy initial values
    memcpy(temp_disparity, disparity_map->data, width * height * sizeof(float));
    memcpy(temp_confidence, disparity_map->confidence, width * height * sizeof(uint8_t));
    // Perform multiple passes of propagation
    for(int pass = 0; pass < 3; pass++){
        for(int y = half_kernel; y < height - half_kernel; y++){
            for(int x = half_kernel; x < width - half_kernel; x++){
                // Skip if we already have high confidence
                if(disparity_map->confidence[y * width + x] > 200) continue;
                float sum_disparity = 0.0f;
                float sum_weights = 0.0f;
                // Compute weighted average of neighboring disparities
                for(int dy = -half_kernel; dy <= half_kernel; dy++){
                    for(int dx = -half_kernel; dx <= half_kernel; dx++){
                        int nx = x + dx;
                        int ny = y + dy;
                        if(nx >= 0 && nx < width && ny >= 0 && ny < height){
                            float weight = disparity_map->confidence[ny * width + nx] / 255.0f;
                            if(weight > 0){
                                sum_disparity += weight * disparity_map->data[ny * width + nx];
                                sum_weights += weight;
                            }
                        }
                    }
                }
                // Update disparity and confidence if we have enough information
                if(sum_weights > 0.1f){
                    temp_disparity[y * width + x] = sum_disparity / sum_weights;
                    temp_confidence[y * width + x] = (uint8_t)(127 * sum_weights);
                }
            }
        }
        // Update the disparity map with propagated values
        memcpy(disparity_map->data, temp_disparity, width * height * sizeof(float));
        memcpy(disparity_map->confidence, temp_confidence, width * height * sizeof(uint8_t));
    }
    
    // Free temporary buffers
    free(temp_disparity);
    free(temp_confidence);
    
    return 1;
}

// Calculate depth from disparity using the formula from the colloquium slides
float computeDepth(float disparity, const CameraParameters* camera_params) {
    if(disparity < 0.1f) return 0.0f;  // Avoid division by very small disparities
    // Formula from slides: Z = (b * f) / d
    return (camera_params->baseline * camera_params->focal_length) / disparity;
}

// Initialize a disparity map
int initDisparityMap(DisparityMap* map, int width, int height){
    if(map == NULL) return 0;
    
    // Allocate memory for disparity values
    map->data = (float*)malloc(width * height * sizeof(float));
    if(map->data == NULL) return 0;
    // Allocate memory for confidence values
    map->confidence = (uint8_t*)malloc(width * height * sizeof(uint8_t));
    if(map->confidence == NULL){
        free(map->data);
        return 0;
    }
    map->width = width;
    map->height = height;
    // Initialize with zeros
    memset(map->data, 0, width * height * sizeof(float));
    memset(map->confidence, 0, width * height * sizeof(uint8_t));
    
    return 1;
}

// Free memory allocated for disparity map
void freeDisparityMap(DisparityMap* map){
    if(map == NULL) return;
    
    free(map->data);
    free(map->confidence);
    
    map->data = NULL;
    map->confidence = NULL;
    map->width = 0;
    map->height = 0;
}

// Save disparity map to a file
int saveDisparityMap(const DisparityMap* map, const char* filename){
    FILE* file = fopen(filename, "wb");
    if(file == NULL){
        printf("Error: Cannot open file %s for writing\n", filename);
        return 0;
    }
    // Write header (width and height)
    fwrite(&map->width, sizeof(int), 1, file);
    fwrite(&map->height, sizeof(int), 1, file);
    // Write disparity data
    fwrite(map->data, sizeof(float), map->width * map->height, file);
    // Write confidence data
    fwrite(map->confidence, sizeof(uint8_t), map->width * map->height, file);

    fclose(file);
    return 1;
}

// Detect obstacles using the disparity map
int detectObstacles(const DisparityMap* disparity_map, const CameraParameters* camera_params, float min_depth, float max_depth, uint8_t* obstacle_map){
    int width = disparity_map->width;
    int height = disparity_map->height;
    
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            int idx = y * width + x;
            float disparity = disparity_map->data[idx];
            uint8_t confidence = disparity_map->confidence[idx];
            // Skip low confidence points
            if(confidence < 50){
                obstacle_map[idx] = 0;
                continue;
            }
            // Calculate depth
            float depth = computeDepth(disparity, camera_params);
            // Check if depth is within the obstacle range
            obstacle_map[idx] = (depth >= min_depth && depth <= max_depth) ? 1 : 0;
        }
    }
    return 1;
}

// Simple feature extraction function
int extractFeatures(const uint8_t* image, int width, int height, Feature* features, int max_features){
    int feature_count = 0;
    srand(42);
    for (int i = 0; i < max_features && feature_count < max_features; i++) {
        float x = 10.0f + (float)(rand() % (width - 20));
        float y = 10.0f + (float)(rand() % (height - 20));
        
        // Set feature position
        features[feature_count].position.x = x;
        features[feature_count].position.y = y;
        
        // Generate descriptor
        features[feature_count].descriptor_size = 32; 
        for (int j = 0; j < features[feature_count].descriptor_size; j++) {
            features[feature_count].descriptor[j] = (float)(rand() % 256) / 255.0f;
        }
        feature_count++;
    }
    return feature_count;
}