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

int main(int argc, char** argv){

    int fps = 60; // in frames per sec
    int frameDelay = 1000/(2*fps); // in millisec 
    int maxDisparity = 128;
    double baseline = 60.0;
    double focalLength = 578.0;
    double minZ = baseline * focalLength / (double)maxDisparity;
    double maxZ = 500.0; // mm
    int rows  = 480;
    int cols  = 640;

    Mat disparityImage = Mat::zeros(rows, cols, CV_8UC1);
    Mat obstacleImage = Mat::zeros(rows, cols, CV_8UC1);
    Mat filteredObstacles = Mat::zeros(rows, cols, CV_8UC1);
    Mat leftFrame, rightFrame, both;
    Mat rectifiedLeft, rectifiedRight;
    Mat medianDisparity, guassianDisparity;
    Mat depthImage = Mat::zeros(rows,cols, CV_8UC1);

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
        stereoDepth(&rectifiedLeft, &rectifiedRight, &disparityImage, maxDisparity, rows, cols);

        // Smooth the depth image
        medianBlur(disparityImage, medianDisparity, 5);
        GaussianBlur(medianDisparity, guassianDisparity, Size(5, 5), 0);

        double disparity;
        double z; // distance from camera
        for(int row = 0; row < rows; row++){
            for(int col = 0; col < cols; col++){
                disparity = (double)(disparityImage.at<unsigned char>(row, col));
                if(disparity > 0) z = baseline * focalLength / disparity;
                else z = 0;
                if(z > minZ && z < maxZ) obstacleImage.at<unsigned char>(row, col) = 255;
                else obstacleImage.at<unsigned char>(row, col) = 0;
            }
        }

        // filter obstacle images to eliminate false matches
        medianBlur(obstacleImage, filteredObstacles, 15);

        // Display
        imshow("depth", guassianDisparity);
        imshow("obstacles", filteredObstacles);

        // Detect obstacles and determine safe direction
        bool leftClear, centerClear, rightClear;
              
        // Generate motor command based on obstacle detection
        char command[8];
        
        // Send command to motor controller if serial port is open
        if(serialPort >= 0) serialPortWrite(command, serialPort);

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


