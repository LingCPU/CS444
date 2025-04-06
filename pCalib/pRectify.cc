#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    
// Load left and right stereo images
    Mat leftImageColor  = imread("nvcamtest_6285_s00_00000.jpg");
    Mat rightImageColor = imread("nvcamtest_6407_s01_00000.jpg");

     cv::Mat leftImage,rightImage;
     cv::cvtColor(leftImageColor,leftImage, cv::COLOR_BGR2GRAY);
     cv::cvtColor(rightImageColor,rightImage, cv::COLOR_BGR2GRAY);

    imshow("left",leftImage);
    imshow("right",rightImage);

    if (leftImage.empty() || rightImage.empty()) {
        cout << "Error: Could not load stereo images!" << endl;
        return -1;
    }

 
// Camera parameters (intrinsic matrices)
       Mat cameraMatrix1 = (Mat_<double>(3,3) << 520.3769098699838, 0, 320.0962070894317,
                                                 0, 721.9725666196721, 209.0549547250552,
                                                  0,0,1);
        Mat cameraMatrix2 = (Mat_<double>(3,3) << 525.6004307791695, 0, 299.4067035390187,
                                                  0, 700.6506914470255, 233.4706305507121,
                                                  0 ,0, 1);
  
        // Distortion coefficients
        Mat distCoeffs1 = (Mat_<double>(1, 5) << 0.0966067969602849, -0.6799990598558086, -0.00877531692018266, 0.007834778660279878, 2.166612417902772);
        Mat distCoeffs2 = (Mat_<double>(1, 5) << 0.0195344493600577,  0.1299319520548544, -0.01194571488397722, 0.005930107627431215, 0.3998885875140788);

//Rotation matrix R:
 Mat R = (Mat_<double>(3,3) << 0.9999984174903006, 0.001479260162221432, 0.0009883350984411189,
                               -0.001484940501796228, 0.9999822417289682, 0.005771583700641334,
                              -0.000979779873477283, -0.00577304218587117, 0.9999828558606991);
//Translation vector T:
Mat T = (Mat_<double>(3,1) << -59.90256900017663,
                              -3.763100080325988,
                              -9.167283490423932);
// Output rectification transforms, projection matrices, and disparity-to-depth mapping matrix
    Mat R1, R2, P1, P2, Q;
    
    // Compute rectification transforms
    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, leftImage.size(), 
                  R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY,-1,leftImage.size());
    
    // Compute undistortion and rectification maps
    Mat map1x, map1y, map2x, map2y;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, leftImage.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, rightImage.size(), CV_32FC1, map2x, map2y);
    
    // Apply rectification
    Mat rectifiedLeft, rectifiedRight;
    remap(leftImage, rectifiedLeft, map1x, map1y, INTER_LINEAR);
    remap(rightImage, rectifiedRight, map2x, map2y, INTER_LINEAR);

    FileStorage fs("lookupTables.xml",FileStorage::WRITE);   
    fs << "Map1x" << map1x; 
    fs << "Map1y" << map1y;
    fs << "Map2x" << map2x; 
    fs << "Map2y" << map2y; 
    fs.release();

    // Display results
    imshow("Rectified Left Image", rectifiedLeft);
    imshow("Rectified Right Image", rectifiedRight); 
//    hconcat(rectifiedLeft,rectifiedRight,combined);

   // Display original images
    imwrite("leftRectified.jpg", rectifiedLeft); 
    imwrite("rightRectified.jpg", rectifiedRight); 
    waitKey(0);
    return 0;
}

