#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Prepare object points (3D points in real world space)
    int board_width = 9;  // Checkerboard width (columns)
    int board_height = 6; // Checkerboard height (rows)
    float square_size = 23.0f; // The size of a square mm 
    int imageWidth  = 640;
    int imageHeight = 480;

    std::vector<cv::Point3f> objectPoints; // 3D points in real world space
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            objectPoints.push_back(cv::Point3f(j * square_size, i * square_size, 0.0f));
        }
    }

    // Vector to store the 2D image points and object points from each image
    std::vector<std::vector<cv::Point2f>> imagePointsLeft,imagePointsRight; // 2D points in the image plane
    std::vector<std::vector<cv::Point3f>> objectPointsAll; // 3D object points
    std::vector<cv::Mat> rvecs, tvecs; // Rotation and translation vectors

    printf("read images on line 26 \n");
    // Read images for calibration
    std::vector<cv::String> fileNamesLeft;
    std::vector<cv::String> fileNamesRight;
    cv::glob("nvcamtest_6285_s00_*.jpg", fileNamesLeft);
    cv::glob("nvcamtest_6407_s01_*.jpg", fileNamesRight); 

    printf("find image  points on line 33 \n");
// find  image points
    for (size_t i = 0; i < fileNamesLeft.size(); i++) {
   
         cv::Mat leftImage = cv::imread(fileNamesLeft[i]);
         cv::Mat rightImage = cv::imread(fileNamesRight[i]);
 
         cv::Mat grayLeft,grayRight;
         cv::cvtColor(leftImage, grayLeft, cv::COLOR_BGR2GRAY);
         cv::cvtColor(rightImage, grayRight, cv::COLOR_BGR2GRAY);
 
        vector<Point2f> cornersL, cornersR;
        bool foundL = findChessboardCorners(grayLeft, Size(board_width, board_height), cornersL);
        bool foundR = findChessboardCorners(grayRight, Size(board_width, board_height), cornersR);

        if (foundL && foundR) {
            // Refine corner positions
            cornerSubPix(grayLeft, cornersL, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01));
            cornerSubPix(grayRight, cornersR, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01));

            // Store the detected points
            objectPointsAll.push_back(objectPoints);
            imagePointsLeft.push_back(cornersL);
            imagePointsRight.push_back(cornersR);

 // Draw corners for visualization
             cv::drawChessboardCorners(grayLeft, cv::Size(board_width, board_height), cornersL, foundL);
             cv::imshow("Chessboard", grayLeft);
             cv::waitKey(500); // Wait for half a second before moving to the next image
             cv::drawChessboardCorners(grayRight, cv::Size(board_width, board_height), cornersR, foundR);
             cv::imshow("Chessboard", grayRight);
             cv::waitKey(1000); // Wait for half a second before moving to the next image

        }
    }


    cv::destroyAllWindows();

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
  

   printf("calibrate camera on line 96 \n");
    // Perform the calibration
    Mat R, T, E, F;
    Size imageSize(imageWidth,imageHeight) ;

    double rms = stereoCalibrate(objectPointsAll, imagePointsLeft, imagePointsRight,
                                 cameraMatrix1, distCoeffs1,
                                 cameraMatrix2, distCoeffs2,
                                 imageSize, R, T, E, F,
                                 CALIB_USE_INTRINSIC_GUESS,
                                 TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 1e-5));
  
    cout << " \n" << endl;
    cout << "Stereo Calibration RMS error: " << rms << endl;
    cout << "Rotation matrix R:\n" << R << endl;
    cout << "Translation vector T:\n" << T << endl;

    cout << " \n" << endl;
    cout << "Left Calibration Matix \n" << cameraMatrix1 << endl;
    cout << "Left Dist Coeffs \n" << distCoeffs1 << endl;

    cout << " \n" << endl;
    cout << "Right Calibration Matix \n" << cameraMatrix2 << endl;
    cout << "Right Dist Coeffs \n" << distCoeffs2 << endl;

    return 0;
}

