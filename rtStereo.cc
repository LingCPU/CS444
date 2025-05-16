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

// Navigation Configuration Parameters
struct NavConfig{
    int baseSpeed = 64;              // Base forward speed (0-128)
    int turnSpeed = 32;              // Speed during turns (0-128)
    int turnStrength = 45;           // Turn angle in degrees from center (90)
    int obstacleThreshold = 10;      // Percentage threshold for obstacle detection
    int regionWidth = 3;             // Number of regions to divide the image into
    double scanHeightPercentage = 0.4; // Percentage of image from bottom to scan
    int stopDistance = 20;           // Minimum safe distance in cm
    int slowDistance = 50;           // Distance at which to slow down in cm
    int maxDisparity = 128;          // Maximum disparity value
    double baseline = 60.0;          // Camera baseline in mm
    double focalLength = 560.0;      // Focal length in pixels
    int commandDelay = 500;          // Delay between commands in milliseconds
};

struct NavCommand{
    string steeringCmd;
    string movementCmd;
    string description;
};

NavCommand decideNavigation(const Mat& obstacleImage, const Mat& disparityImage, 
                        const NavConfig& config, int rows, int cols){
    
    // Define regions of interest for obstacle detection
    int regionWidth = cols / config.regionWidth;
    int scanStartRow = rows * (1.0 - config.scanHeightPercentage);
    
    // Calculate obstacle detection thresholds
    int pixelThreshold = regionWidth * (rows - scanStartRow) * (config.obstacleThreshold / 100.0);
    
    // Arrays to store obstacle counts for each region
    vector<int> obstacleCounts(config.regionWidth, 0);
    vector<double> minDistances(config.regionWidth, 99999.0);
    
    // Scan for obstacles in each region
    for(int region = 0; region < config.regionWidth; region++){
        int regionStart = region * regionWidth;
        int regionEnd = (region + 1) * regionWidth;
        
        // Count obstacles in this region
        for(int row = scanStartRow; row < rows; row++){
            for(int col = regionStart; col < regionEnd && col < cols; col++){
                if(obstacleImage.at<unsigned char>(row, col) > 0){
                    obstacleCounts[region]++;
                    
                    // Calculate distance for this pixel from disparity
                    double disparity = disparityImage.at<unsigned char>(row, col);
                    if(disparity > 0){
                        double distance = config.baseline * config.focalLength / disparity;
                        if(distance < minDistances[region]) minDistances[region] = distance;
                    }
                }
            }
        }
    }

    // Determine which regions are obstructed
    vector<bool> obstructed(config.regionWidth, false);
    for(int region = 0; region < config.regionWidth; region++){
        obstructed[region] = (obstacleCounts[region] > pixelThreshold) || (minDistances[region] < config.stopDistance);
    }

    bool leftObstructed = obstructed[0];
    bool centerObstructed = (config.regionWidth >= 3) ? obstructed[1] : false;
    bool rightObstructed = obstructed[config.regionWidth - 1];
    
    // turn angles
    int leftTurnAngle = 90 - config.turnStrength;
    int rightTurnAngle = 90 + config.turnStrength;
    
    // Formatting
    char centerCmd[8], leftCmd[8], rightCmd[8];
    char forwardCmd[8], slowForwardCmd[8], stopCmd[8];
    sprintf(centerCmd, "STR090\n");
    sprintf(leftCmd, "STR%03d\n", leftTurnAngle);
    sprintf(rightCmd, "STR%03d\n", rightTurnAngle);
    sprintf(forwardCmd, "FWD%03d\n", config.baseSpeed);
    sprintf(slowForwardCmd, "FWD%03d\n", config.baseSpeed / 2);
    sprintf(stopCmd, "STP000\n");
    
    // Decision tree
    NavCommand command;

    if(!leftObstructed && !centerObstructed && !rightObstructed){
        command.steeringCmd = centerCmd;
        command.movementCmd = forwardCmd;
    } else if(!centerObstructed){
        if(config.regionWidth >= 3 && minDistances[1] < config.slowDistance) {
            command.steeringCmd = centerCmd;
            command.movementCmd = slowForwardCmd;
        } else{
            command.steeringCmd = centerCmd;
            command.movementCmd = forwardCmd;
        }
    } else if(!leftObstructed && !rightObstructed){
        if(minDistances[0] >= minDistances[config.regionWidth-1]){
            // Left has more space
            command.steeringCmd = leftCmd;
            command.movementCmd = forwardCmd;
        } else{
            // Right has more space
            command.steeringCmd = rightCmd;
            command.movementCmd = forwardCmd;
        }
    } else if(!leftObstructed){
        // Only left is clear
        command.steeringCmd = leftCmd;
        command.movementCmd = forwardCmd;
    } else if(!rightObstructed){
        // Only right is clear
        command.steeringCmd = rightCmd;
        command.movementCmd = forwardCmd;
    } else{
        command.steeringCmd = centerCmd;
        command.movementCmd = stopCmd;
    }

    return command;
}

int main(int argc, char** argv){

    NavConfig config;
    config.baseSpeed = 80;             
    config.turnSpeed = 40;
    config.turnStrength = 45;         
    config.obstacleThreshold = 10;     
    config.regionWidth = 3;             
    config.scanHeightPercentage = 0.4;  
    config.stopDistance = 200;        
    config.slowDistance = 500;        
    config.maxDisparity = 128;
    config.baseline = 60.0;            
    config.focalLength = 560.0;    
    config.commandDelay = 500; 
    
    int fps = 30;
    int frameDelay = 1000/fps;
    int rows = 480;
    int cols = 640;
    
    double minZ = config.baseline * config.focalLength / (double)config.maxDisparity;
    double maxZ = 500.0; // mm

    Mat disparityImage = Mat::zeros(rows, cols, CV_8UC1);
    Mat obstacleImage = Mat::zeros(rows, cols, CV_8UC1);
    Mat filteredObstacles = Mat::zeros(rows, cols, CV_8UC1);
    Mat leftFrame, rightFrame, both;
    Mat rectifiedLeft, rectifiedRight;
    Mat medianDisparity, guassianDisparity;
    Mat depthImage = Mat::zeros(rows,cols, CV_8UC1);

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

    float offset = 0.0;
    float currentRow;
    for(int row = 0; row < rows; row++){
        for(int col = 0; col < cols; col++){
            currentRow = map2y.at<float>(row, col);
            if(currentRow + offset < 0 || currentRow + offset > rows) map2y.at<float>(row, col) = currentRow;
            else map2y.at<float>(row, col) = currentRow + offset;
        }
    }

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
        remap(leftFrame, rectifiedLeft, map1x, map1y, INTER_LINEAR);
        remap(rightFrame, rectifiedRight, map2x, map2y, INTER_LINEAR);

        // Compute depth image
        stereoDepth(&rectifiedLeft, &rectifiedRight, &disparityImage, config.maxDisparity, rows, cols);

        // Smooth the depth image
        medianBlur(disparityImage, medianDisparity, 5);
        GaussianBlur(medianDisparity, guassianDisparity, Size(5, 5), 0);

        for(int row = 0; row < rows; row++){
            for(int col = 0; col < cols; col++){
                double disparity = (double)(disparityImage.at<unsigned char>(row, col));
                if(disparity > 0){
                    double distance = config.baseline * config.focalLength / disparity;
                    if(distance > minZ && distance < maxZ) obstacleImage.at<unsigned char>(row, col) = 255;
                    else obstacleImage.at<unsigned char>(row, col) = 0;
                } else obstacleImage.at<unsigned char>(row, col) = 0;
            }
        }

        // filter obstacle images to eliminate false matches
        medianBlur(obstacleImage, filteredObstacles, 11);
        dilate(filteredObstacles, filteredObstacles, Mat(), Point(-1, -1), 2);

        NavCommand command = decideNavigation(filteredObstacles, disparityImage, config, rows, cols);

        // Display
        imshow("depth", guassianDisparity);
        imshow("obstacles", filteredObstacles);
        
        // Send commands to motor controller if serial port is open
        if(serialPort >= 0){
            serialPortWrite(command.steeringCmd.c_str(), serialPort);
            serialPortWrite(command.movementCmd.c_str(), serialPort);
        }
        // display depth map
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


