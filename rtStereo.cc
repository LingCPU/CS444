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
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include "stereoDepth.h"

using namespace cv;
using namespace std;

int serialPortOpen(){
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    if(serial_port == -1){
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(serial_port, &tty) != 0){
        perror("Error getting current serial port settings");
        return -1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICRNL | INLCR);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    if(tcsetattr(serial_port, TCSANOW, &tty) != 0){
        perror("Error setting serial port attributes");
        return -1;
    }

    return serial_port;
}

int serialPortWrite(const char *data, int serial_port){
    int n = write(serial_port, data, strlen(data));
    if(n < 0){
        perror("Error writing to serial port");
        return -1;
    }
    return n;
}

int serialPortClose(int serial_port){
    return close(serial_port);
}


void detectObstacles(const Mat& depthImage, bool& leftClear, bool& centerClear, bool& rightClear, 
                     double obstacleThreshold, Mat& obstacleImage) {
    int rows = depthImage.rows;
    int cols = depthImage.cols;
    
    // Create a color visualization of the obstacles
    obstacleImage = Mat(rows, cols, CV_8UC3, Scalar(0, 0, 0));
    
    // Define regions, counters, and horizon
    int leftStart = 0;
    int leftEnd = cols / 3;
    int centerStart = cols / 3;
    int centerEnd = 2 * cols / 3;
    int rightStart = 2 * cols / 3;
    int rightEnd = cols;
    
    // Adjust horizon line to focus on more relevant part of the image
    int horizonLine = rows * 2 / 3;  // Lower in the image (closer to robot)
    
    // Count obstacles in different distance zones
    int nearZone = obstacleThreshold / 3;       // Very close obstacles
    int midZone = obstacleThreshold * 2 / 3;    // Medium distance obstacles
    
    // Counters for obstacles
    int leftNearObstacles = 0, leftMidObstacles = 0, leftFarObstacles = 0;
    int centerNearObstacles = 0, centerMidObstacles = 0, centerFarObstacles = 0;
    int rightNearObstacles = 0, rightMidObstacles = 0, rightFarObstacles = 0;
    
    // Total pixels in each region (for calculating density)
    int leftTotal = (leftEnd - leftStart) * (rows - horizonLine);
    int centerTotal = (centerEnd - centerStart) * (rows - horizonLine);
    int rightTotal = (rightEnd - rightStart) * (rows - horizonLine);
    
    // Detect obstacles
    for(int y = horizonLine; y < rows; y++) {
        for(int x = 0; x < cols; x++) {
            // Skip pixels with no depth info
            uchar depth = depthImage.at<uchar>(y, x);
            if(depth == 0) continue;
            
            // Check if pixel is within obstacle range
            if(depth < obstacleThreshold) {
                // Determine color based on depth (closer = more red)
                Vec3b color;
                if(depth < nearZone) {
                    color = Vec3b(0, 0, 255);  // Red for very close obstacles
                } else if(depth < midZone) {
                    color = Vec3b(0, 165, 255);  // Orange for medium obstacles
                } else {
                    color = Vec3b(0, 255, 255);  // Yellow for farther obstacles
                }
                
                // Set pixel in obstacle image
                obstacleImage.at<Vec3b>(y, x) = color;
                
                // Count obstacle in the appropriate region and distance
                if(x >= leftStart && x < leftEnd) {
                    if(depth < nearZone) leftNearObstacles++;
                    else if(depth < midZone) leftMidObstacles++;
                    else leftFarObstacles++;
                } 
                else if(x >= centerStart && x < centerEnd) {
                    if(depth < nearZone) centerNearObstacles++;
                    else if(depth < midZone) centerMidObstacles++;
                    else centerFarObstacles++;
                } 
                else if(x >= rightStart && x < rightEnd) {
                    if(depth < nearZone) rightNearObstacles++;
                    else if(depth < midZone) rightMidObstacles++;
                    else rightFarObstacles++;
                }
            }
        }
    }
    
    // Calculate obstacle density with weighted importance for closer obstacles
    float weightNear = 3.0;  // Near obstacles count more
    float weightMid = 1.5;   // Medium obstacles have some importance
    float weightFar = 1.0;   // Far obstacles count normally
    
    double leftDensity = (leftNearObstacles * weightNear + leftMidObstacles * weightMid + leftFarObstacles * weightFar) / leftTotal;
    double centerDensity = (centerNearObstacles * weightNear + centerMidObstacles * weightMid + centerFarObstacles * weightFar) / centerTotal;
    double rightDensity = (rightNearObstacles * weightNear + rightMidObstacles * weightMid + rightFarObstacles * weightFar) / rightTotal;
    
    // Adaptive threshold based on overall obstacle density
    double totalDensity = (leftDensity + centerDensity + rightDensity) / 3.0;
    double densityThreshold = 0.05 + (totalDensity * 0.1);  // Base + adaptive component
    densityThreshold = min(densityThreshold, 0.15);  // Cap at 0.15
    
    // Determine which regions are clear
    leftClear = (leftDensity < densityThreshold);
    centerClear = (centerDensity < densityThreshold);
    rightClear = (rightDensity < densityThreshold);
    
    // Draw region markers on the obstacle image
    line(obstacleImage, Point(leftEnd, 0), Point(leftEnd, rows-1), Scalar(255, 255, 255), 2);
    line(obstacleImage, Point(centerEnd, 0), Point(centerEnd, rows-1), Scalar(255, 255, 255), 2);
    line(obstacleImage, Point(0, horizonLine), Point(cols-1, horizonLine), Scalar(255, 255, 255), 2);
    
    // Display density values and thresholds for debugging
    putText(obstacleImage, format("L: %.3f %s", leftDensity, leftClear ? "CLEAR" : "BLOCKED"), 
            Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
    putText(obstacleImage, format("C: %.3f %s", centerDensity, centerClear ? "CLEAR" : "BLOCKED"), 
            Point(cols/3 + 10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
    putText(obstacleImage, format("R: %.3f %s", rightDensity, rightClear ? "CLEAR" : "BLOCKED"), 
            Point(2*cols/3 + 10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
}

void generateMotorCommand(bool leftClear, bool centerClear, bool rightClear, 
                          char* command, int baseSpeed){
    // Default is to stop if no clear path
    strcpy(command, "STP000\n");
    
    // Decision tree for navigation
    if(centerClear) sprintf(command, "FWD%03d\n", baseSpeed);
    else if(leftClear && !rightClear) strcpy(command, "STR045\n");
    else if(!leftClear && rightClear) strcpy(command, "STR135\n");
    else if(leftClear && rightClear) strcpy(command, "STR045\n");
}

int main(int argc, char** argv){

    int fps = 10; // in frames per sec
    int frameDelay = 1000/(2*fps); // in millisec 
    double maxDistance = 5000.0; // mm
    int rows  = 480;
    int cols  = 640;
    Mat depthImage = Mat::zeros(rows,cols, CV_8UC1);
    Mat obstacleImage = Mat::zeros(rows, cols, CV_8UC1);
    double obstacleThreshold = 100;
    int baseSpeed = 64;

    // serial port things:
    int serialPort = serialPortOpen();
    if(serialPort < 0) cout << "Failed to open serial port";
    else cout << "Opened serial port successfully";


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

    if(serialPort >= 0) serialPortWrite("STP000\n", serialPort);
        
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

        // Detect obstacles and determine safe direction
        bool leftClear, centerClear, rightClear;
        detectObstacles(medianFiltered, leftClear, centerClear, rightClear, obstacleThreshold, obstacleImage);
        
        // Generate motor command based on obstacle detection
        char command[8];
        generateMotorCommand(leftClear, centerClear, rightClear, command, baseSpeed);
        
        // Send command to motor controller if serial port is open
        if(serialPort >= 0) serialPortWrite(command, serialPort);

        // display depth map
        imshow("Depth",medianFiltered);
        hconcat(rectifiedLeft, rectifiedRight,both);
        imshow("Left and Right",both);
    
        int key = waitKey(frameDelay);
        if(key == 'q') break;
        
        // Emergency stop on spacebar
        if(key == ' ' && serialPort >= 0) {
            serialPortWrite("STP000\n", serialPort);
            cout << "Emergency stop triggered!" << endl;
        }
    }

    if(serialPort >= 0){
        serialPortWrite("STP000\n", serialPort);
        serialPortClose(serialPort);
        cout << "Robot stopped and serial port closed." << endl;
    }

    // Release resources
    capL.release();
    capR.release();
    destroyAllWindows();

    return 0;
}


