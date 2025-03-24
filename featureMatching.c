#include "featureMatching.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

// Parameters for feature detection
#define FAST_THRESHOLD 20       // Intensity difference threshold for corner detection
#define FAST_RADIUS 3           // Radius for checking surrounding pixels
#define NMS_RADIUS 5            // Non-maximum suppression radius
#define DESCRIPTOR_SIZE 32      // Size of binary descriptor in bytes (32 bytes = 256 bits)
#define DESCRIPTOR_PATCH_SIZE 9 // Size of patch to compute descriptor (9x9)

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

// Check if a pixel is a corner by testing surrounding pixels in a circle
static int isCorner(const uint8_t* image, int width, int height, int x, int y, int threshold) {
    // If we're too close to the edge, it's not a corner
    if(x < FAST_RADIUS || x >= width - FAST_RADIUS || y < FAST_RADIUS || y >= height - FAST_RADIUS) return 0;

    // Center pixel value
    uint8_t center = image[y * width + x];

    // Test points on a circle around the pixel
    // We'll check 8 points (N, NE, E, SE, S, SW, W, NW) instead of 16 for efficiency
    const int circle_x[8] = {0, 1, 2, 1, 0, -1, -2, -1};
    const int circle_y[8] = {-2, -1, 0, 1, 2, 1, 0, -1};

    // Count consecutive pixels that are significantly brighter or darker
    int consecutive_bright = 0;
    int consecutive_dark = 0;
    int max_consecutive_bright = 0;
    int max_consecutive_dark = 0;
    
    for(int i = 0; i < 16; i++){ // Go around twice to handle wrapping
        int idx = i % 8;
        int px = x + circle_x[idx];
        int py = y + circle_y[idx];
        uint8_t value = image[py * width + px];
        
        // Check if significantly brighter
        if(value > center + threshold){
            consecutive_bright++;
            consecutive_dark = 0;
        } else if(value < center - threshold){ // Check if significantly darker
            consecutive_dark++;
            consecutive_bright = 0;
        } else{
            consecutive_bright = 0;
            consecutive_dark = 0;
        }
        // Update maximum consecutive counts
        if(consecutive_bright > max_consecutive_bright) max_consecutive_bright = consecutive_bright;
        if(consecutive_dark > max_consecutive_dark) max_consecutive_dark = consecutive_dark;
    }
    // If we have at least 3 consecutive pixels that are brighter or darker,
    // consider this a corner
    return (max_consecutive_bright >= 3 || max_consecutive_dark >= 3);
}


// Compute corner score for non-maximum suppression
// should return the sum of absolute differences from center pixel
static int cornerScore(const uint8_t* image, int width, int height, int x, int y){
    // Center pixel value
    uint8_t center = image[y * width + x];
    int score = 0;
    
    // Same circle points as in isCorner function
    const int circle_x[8] = {0, 1, 2, 1, 0, -1, -2, -1};
    const int circle_y[8] = {-2, -1, 0, 1, 2, 1, 0, -1};
    
    // Sum up absolute differences
    for(int i = 0; i < 8; i++){
        int px = x + circle_x[i];
        int py = y + circle_y[i];
        uint8_t value = image[py * width + px];
        score += abs((int)value - (int)center);
    }
    return score;
}


// Compute a simple binary descriptor for a feature point
static void computeDescriptor(const uint8_t* image, int width, int height, int x, int y, float* descriptor){
    // Initialize descriptor
    memset(descriptor, 0, DESCRIPTOR_SIZE * sizeof(float));

    // Check if we have enough space to compute descriptor
    if(x < DESCRIPTOR_PATCH_SIZE/2 || x >= width - DESCRIPTOR_PATCH_SIZE/2 || y < DESCRIPTOR_PATCH_SIZE/2 || y >= height - DESCRIPTOR_PATCH_SIZE/2) return;

    // Seed for reproducible pair selection
    srand(42);
    
    // Generate pairs for binary tests
    // Each byte will store 8 binary tests
    for(int byte = 0; byte < DESCRIPTOR_SIZE; byte++){
        uint8_t byte_value = 0;
        for(int bit = 0; bit < 8; bit++){
            // Generate two random points in the patch
            int x1 = (rand() % DESCRIPTOR_PATCH_SIZE) - DESCRIPTOR_PATCH_SIZE/2;
            int y1 = (rand() % DESCRIPTOR_PATCH_SIZE) - DESCRIPTOR_PATCH_SIZE/2;
            int x2 = (rand() % DESCRIPTOR_PATCH_SIZE) - DESCRIPTOR_PATCH_SIZE/2;
            int y2 = (rand() % DESCRIPTOR_PATCH_SIZE) - DESCRIPTOR_PATCH_SIZE/2;
            
            // Sample pixel values
            uint8_t value1 = image[(y + y1) * width + (x + x1)];
            uint8_t value2 = image[(y + y2) * width + (x + x2)];
            
            // Set bit if first point is brighter than second
            if(value1 < value2) byte_value |= (1 << bit);
        }
        // Store byte as a float in descriptor (for compatibility with existing code)
        descriptor[byte] = (float)byte_value;
    }
}

int extractFeatures(const uint8_t* image, int width, int height, Feature* features, int max_features){
    // Temporary storage for corners
    int* corner_x = (int*)malloc(width * height * sizeof(int));
    int* corner_y = (int*)malloc(width * height * sizeof(int));
    int* corner_scores = (int*)malloc(width * height * sizeof(int));
    
    if(corner_x == NULL || corner_y == NULL || corner_scores == NULL){
        free(corner_x);
        free(corner_y);
        free(corner_scores);
        return 0;
    }
    // Detect corners
    int corner_count = 0;
    for(int y = FAST_RADIUS; y < height - FAST_RADIUS; y++){
        for(int x = FAST_RADIUS; x < width - FAST_RADIUS; x++){
            if(isCorner(image, width, height, x, y, FAST_THRESHOLD)){
                // Store corner
                corner_x[corner_count] = x;
                corner_y[corner_count] = y;
                corner_scores[corner_count] = cornerScore(image, width, height, x, y);
                corner_count++;
                // Check if we found too many corners
                if(corner_count >= width * height) break;
            }
        }
        // Break if we found too many corners
        if(corner_count >= width * height) break;
    }
    printf("Detected %d corners\n", corner_count);

    // Apply non-maximum suppression
    int feature_count = 0;
    for(int i = 0; i < corner_count && feature_count < max_features; i++){
        int x = corner_x[i];
        int y = corner_y[i];
        int score = corner_scores[i];

        // Check if this corner has maximum score in its neighborhood
        int is_maximum = 1;
        for(int j = 0; j < corner_count; j++){
            if(i != j){
                int dx = corner_x[j] - x;
                int dy = corner_y[j] - y;
                int distance_squared = dx*dx + dy*dy;
                // If another corner is in the neighborhood and has higher score
                if(distance_squared < NMS_RADIUS*NMS_RADIUS && corner_scores[j] > score){
                    is_maximum = 0;
                    break;
                }
            }
        }
        // If this corner has maximum score, add it as a feature
        if(is_maximum){
            features[feature_count].position.x = (float)x;
            features[feature_count].position.y = (float)y;
            features[feature_count].descriptor_size = DESCRIPTOR_SIZE;
            // Compute descriptor
            computeDescriptor(image, width, height, x, y, features[feature_count].descriptor);
            feature_count++;
        }
    }
    // Free temporary storage
    free(corner_x);
    free(corner_y);
    free(corner_scores);

    printf("After non-maximum suppression: %d features\n", feature_count);
    return feature_count;
}

// Calculate similarity between two descriptors
float calculateDescriptorSimilarity(const float* desc1, const float* desc2, int size){
    int hamming_distance = 0;
    // Calculate Hamming distance
    for(int i = 0; i < size; i++){
        uint8_t byte1 = (uint8_t)desc1[i];
        uint8_t byte2 = (uint8_t)desc2[i];
        uint8_t xor_result = byte1 ^ byte2;
        // Count bits set in xor_result (Brian Kernighan's algorithm)
        while(xor_result){
            xor_result &= (xor_result - 1);
            hamming_distance++;
        }
    }
    // Convert distance to similarity (lower distance = higher similarity)
    // Maximum Hamming distance for 256-bit descriptor is 256
    float similarity = 1.0f - (float)hamming_distance / (size * 8);
    return similarity;
}