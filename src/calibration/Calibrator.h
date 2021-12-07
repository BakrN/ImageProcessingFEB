#pragma once 
#include "../StereoCamera.h"
#include <opencv2/core.hpp>
#include <iostream> 
#include <vector>
class Calibrator { 
    // THIS CLASS CALIBRATES CAMERAS USING THE CHESSBOARD 
    private: 
        std::shared_ptr<StereoCamera> m_Camera; 
        std::string m_FolderPath; 
        std::vector<cv::Point3f > m_CheckerBoardPoints;
        std::vector<std::vector<cv::Point2f> > m_Imgpoints;
        int CHECKERBOARD[2] = {6,9}; //checkerboard size;  
   
    public: 
    Calibrator(); 
    Calibrator(const std::shared_ptr<StereoCamera>& Camera); 
    ~Calibrator(); 

    //Calibrator(DualCamera* Camera, const std::string& folderpath,char new_calibration=true); 
    void Calibrate(char new_calibration=true);
    void Calibrate(const std::shared_ptr<StereoCamera>& Camera, char new_calibration=true); 
    
}; 