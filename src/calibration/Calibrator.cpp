#include "Calibrator.h"
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp> 
#include <iostream>
#include <filesystem>
#include <ros/ros.h> 
Calibrator::Calibrator()
{
     // Defining the world coordinates for Checkerboard image 3D points

    for (int i{0}; i < CHECKERBOARD[1]; i++)
    {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            m_CheckerBoardPoints.push_back(cv::Point3f(j, i, 0));
    }
}
Calibrator::Calibrator(const std::shared_ptr<StereoCamera>& Camera){
      //initializing checkerboard 3d points
    for (int i{0}; i < CHECKERBOARD[1]; i++)
    {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            m_CheckerBoardPoints.push_back(cv::Point3f(j, i, 0));
    }
    m_Camera = Camera; 
}
Calibrator::~Calibrator(){

}
void Calibrator::Calibrate(char new_calibration){
    // start taking images
    if(!m_Camera){
        // log error that cam wasn't initialized then return 
        return ;
    }
    std::string CurrentDir = std::filesystem::current_path().string();
   
    if (new_calibration)
    {
         cv::namedWindow("Calibrationwindow"); 

   
    while(cv::waitKey() != 's'); 
        // empty directory
        if(!std::filesystem::is_empty(std::filesystem::path(CurrentDir + "/CalibrationImages"))){
        for (const auto &entry : std::filesystem::directory_iterator(CurrentDir + "/CalibrationImages"))
            std::filesystem::remove_all(entry.path());
        }
        ROS_INFO("Emptied directory"); 
        int snapshot_count = 0; 
        while (snapshot_count < 30)
        {

            m_Camera->AquireImage();
             cv::imshow("Calibrationwindow",m_Camera->GetVideoImage()); 
             
            // delay time 
            if(cv::waitKey(1) == int('n')){
                ROS_INFO("Got images**************************"); 
            
            ROS_INFO("written to  directory"); 
                cv::imwrite(CurrentDir + "/CalibrationImages/LEFT" + std::to_string(snapshot_count + 1) + ".jpg", m_Camera->GetLeftImage());
                cv::imwrite(CurrentDir + "/CalibrationImages/RIGHT" + std::to_string(snapshot_count + 1) + ".jpg", m_Camera->GetRightImage());
                snapshot_count++; 
            }
        }
        cv::destroyWindow("Calibrationwindow"); 
    }
    else{
        //TODO:  try to read from file  
        //StereoCameraParams.xml
        
        return ; 
    }
    cv::namedWindow("CalibrationLeft");
    cv::namedWindow("CalibrationRight");
    // start calibration
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objpointsl; 
    std::vector<std::vector<cv::Point3f>> objpointsr; 

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> l_imgpoints; 
    std::vector<std::vector<cv::Point2f>> r_imgpoints; 

   
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> LeftImages;
    std::vector<cv::String> RightImages;
    // Path of the folder containing checkerboard images

    cv::glob(CurrentDir + "/CalibrationImages/LEFT*.jpg", LeftImages);
    cv::glob(CurrentDir + "/CalibrationImages/RIGHT*.jpg", RightImages);

    cv::Mat frame;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i{0}; i < LeftImages.size(); i++)
    {
       
        
        frame = cv::imread(LeftImages[i], cv::IMREAD_GRAYSCALE);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true

        //Harris algorithm 
        success = cv::findChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checsker board
        */
        if (success)
        {

            cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(frame, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

        objpointsl.push_back(m_CheckerBoardPoints); 
            l_imgpoints.push_back(corner_pts);
        }

        cv::imshow("CalibrationLeft", frame);

        frame = cv::imread(RightImages[i],cv::IMREAD_GRAYSCALE);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true

        //Harris algorithm 
        success = cv::findChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checsker board
        */
        if (success)
        {

            cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(frame, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

        objpointsr.push_back(m_CheckerBoardPoints); 
            r_imgpoints.push_back(corner_pts);
        }

        cv::imshow("CalibrationRight", frame);

        while(cv::waitKey(0)!= int('n'));
    }

    cv::destroyAllWindows();

    cv::Mat R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */

    cv::calibrateCamera(objpointsl, l_imgpoints, cv::Size(frame.rows, frame.cols), m_Camera->GetLeftIntrinsicParams(), m_Camera->GetLeftDistortionCoeff(),R, T);
    cv::calibrateCamera(objpointsr, r_imgpoints, cv::Size(frame.rows, frame.cols), m_Camera->GetRigthIntrinsicParams(), m_Camera->GetRightDistortionCoeff(), R, T);
    // writing the results into file 
     
    cv::FileStorage ParamsStorage(CurrentDir+"/StereoCameraParams.xml",cv::FileStorage::Mode::WRITE ); 
    ParamsStorage.write("Left_Camera_Intrinsic_Parameters", m_Camera->GetLeftIntrinsicParams()); 
    ParamsStorage.write("Left_Camera_Distortion_Coefficients", m_Camera->GetLeftDistortionCoeff()); 
    ParamsStorage.write("Rigth_Camera_Intrinsic_Parameters", m_Camera->GetRigthIntrinsicParams()); 
    ParamsStorage.write("Right_Camera_Distortion_Coefficients", m_Camera->GetRightDistortionCoeff()); 
    ParamsStorage.release(); 
}

void Calibrator::Calibrate(const std::shared_ptr<StereoCamera>& Camera, char new_calibration)
{
    m_Camera = Camera; 
    this->Calibrate(new_calibration); 
}

/*
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

//https://docs.opencv.org/3.4.15/dc/dbb/tutorial_py_calibration.html
// Defining the dimensions of checkerboard



int CHECKERBOARD[2]{ 6,9 };
using namespace cv; 
using namespace std; 
int main()
{

    Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;

    /* // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }


    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "A:/Checkerboard_mobile_imgs/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i{ 0 }; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH |cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
       
        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checsker board
        */
/* if (success)
        {
            
            cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */
/* cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "internal cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distortion coefficients : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;

    return 0;*/
/* return 0; 
}*/
