#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <string>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "stereoDepth.h"

using namespace cv;
using namespace std;

void detectObstacles(const Mat& depthImage, bool& leftClear, bool& centerClear, bool& rightClear, 
                     double obstacleThreshold, Mat& obstacleImage){
                        
    int rows = depthImage.rows;
    int cols = depthImage.cols;
    obstacleImage = Mat::zeros(rows, cols, CV_8UC1);
    
    // Define regions, counters, horizon
    int leftStart = 0;
    int leftEnd = cols / 3;
    int centerStart = cols / 3;
    int centerEnd = 2 * cols / 3;
    int rightStart = 2 * cols / 3;
    int rightEnd = cols;
    int horizonLine = rows / 2;
    int leftObstacles = 0;
    int centerObstacles = 0;
    int rightObstacles = 0;
    
    // Count total pixels in each region
    int leftTotal = (leftEnd - leftStart) * (rows - horizonLine);
    int centerTotal = (centerEnd - centerStart) * (rows - horizonLine);
    int rightTotal = (rightEnd - rightStart) * (rows - horizonLine);
    
    // Detect obstacles in depth image
    for(int y = horizonLine; y < rows; y++){
        for(int x = 0; x < cols; x++){
            // Get the depth value
            uchar depth = depthImage.at<uchar>(y, x);
            
            // If the pixel has a non-zero depth value and is closer than threshold
            // The lower the depth value, the closer the object (we invert the check)
            if(depth > 0 && depth < obstacleThreshold){
                // Mark as obstacle in the obstacle image
                obstacleImage.at<uchar>(y, x) = 255;
                
                // Count obstacle in the appropriate region
                if(x >= leftStart && x < leftEnd) leftObstacles++;
                else if(x >= centerStart && x < centerEnd) centerObstacles++;
                else if(x >= rightStart && x < rightEnd) rightObstacles++;
            }
        }
    }
    
    // Calculate obstacle density for each region
    double leftDensity = (double)leftObstacles / leftTotal;
    double centerDensity = (double)centerObstacles / centerTotal;
    double rightDensity = (double)rightObstacles / rightTotal;

    double densityThreshold = 0.1; // 10% obstacle density
    
    // Figure which region/side is best
    leftClear = (leftDensity < densityThreshold);
    centerClear = (centerDensity < densityThreshold);
    rightClear = (rightDensity < densityThreshold);
    
    // Draw region markers on the obstacle image
    line(obstacleImage, Point(leftEnd, 0), Point(leftEnd, rows-1), Scalar(128), 2);
    line(obstacleImage, Point(centerEnd, 0), Point(centerEnd, rows-1), Scalar(128), 2);
    line(obstacleImage, Point(0, horizonLine), Point(cols-1, horizonLine), Scalar(128), 2);   
}

int main(int argc, char** argv){

    int fps = 60; // in frames per sec
    int frameDelay = 1000/(2*fps); // in millisec 
    double maxDistance = 5000.0; // mm
    int rows  = 480;
    int cols  = 640;
    Mat depthImage = Mat::zeros(rows,cols, CV_8UC1);

    // Read rectification lookup tables
    Mat map1x,map1y,map2x,map2y;
    FileStorage fs("lookupTables.xml",FileStorage::READ);
    fs["Map1x"]>>map1x;
    fs["Map1y"]>>map1y;
    fs["Map2x"]>>map2x;
    fs["Map2y"]>>map2y;
    fs.release();

    if( map1x.empty()) cout << "Empty 1x lookup table"<<endl;
    if( map1y.empty()) cout << "Empty 1y lookup table"<<endl;
    if( map2x.empty()) cout << "Empty 2x lookup table"<<endl;
    if( map2y.empty()) cout << "Empty 2y lookup table"<<endl;

    string left_cam_pipeline  = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, framerate="+to_string(fps)+
                                "/1 ! nvvidconv flip-method=rotate-180 ! video/x-raw, format=GRAY8 !  appsink drop=1";

    string right_cam_pipeline = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, framerate="+to_string(fps)+
                                "/1 ! nvvidconv flip-method=rotate-180 ! video/x-raw, format=GRAY8 !  appsink drop =1";
        
    // Open both cameras
        VideoCapture capL(left_cam_pipeline, CAP_GSTREAMER);
        VideoCapture capR(right_cam_pipeline,CAP_GSTREAMER);


    if(!capL.isOpened() || !capR.isOpened()){
        cerr << "Error: Could not open stereo cameras." << endl;
        return -1;
    }


    Mat leftFrame, rightFrame;

    cout << " width \n" << capL.get(CAP_PROP_FRAME_WIDTH)<<endl;
    cout << " height  \n" << capL.get(CAP_PROP_FRAME_HEIGHT)<<endl;
    cout << " format \n" << capL.get(CAP_PROP_FORMAT)<<endl;
    cout << " fps \n" << capL.get(CAP_PROP_FPS)<<endl;
        
    while(true){
        // Capture frames from both cameras
        capL >> leftFrame;
        capR >> rightFrame;
        if(leftFrame.empty() || rightFrame.empty()){
            cerr << "Error: Empty frame detected!" << endl;
            break;
        }

        // Apply rectification
        Mat rectifiedLeft, rectifiedRight, both;
        remap(leftFrame, rectifiedLeft, map1x, map1y, INTER_LINEAR);
        remap(rightFrame, rectifiedRight, map2x, map2y, INTER_LINEAR);

        // Compute depth image
        stereoDepth(&rectifiedLeft, &rectifiedRight, &depthImage, maxDistance, rows, cols);

        // Smooth the depth image
        Mat medianFiltered;
        medianBlur(depthImage, medianFiltered, 3);

        // display depth map
        imshow("Depth",medianFiltered);
        hconcat(rectifiedLeft, rectifiedRight,both);
        imshow("Left and Right",both);
    
        waitKey(frameDelay) ;
    }

    // Release resources
    capL.release();
    capR.release();
    destroyAllWindows();

    return 0;
}


