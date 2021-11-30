#pragma once 
#include "camera/DxImageProc.h"
#include "camera/GxIAPI.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

class DualCamera{
    private:
        cv::Mat m_Frames[2]; 
        cv::Mat m_InternalParams[2]; 
        int m_DistortionCoeff[2][5]; // 1st row - x 2nd row - y 

    public: 
        DualCamera() = default; 
        DualCamera(int width, int height); 
        ~DualCamera() = default;  
        void SetImage(const cv::Mat& DualImage) {
 
              m_Frames[0] = DualImage(cv::Range(0, DualImage.rows - 1), cv::Range(0, (DualImage.cols/2) - 1)); // 
              m_Frames[1] = DualImage(cv::Range(0, DualImage.rows - 1), cv::Range(DualImage.cols/2, DualImage.cols- 1));
        }
        void SetImage(const cv::Mat& DualImage, int cv_conversion_code) {
              m_Frames[0] = DualImage(cv::Range(0, DualImage.rows - 1), cv::Range(0, (DualImage.cols/2) - 1));
              m_Frames[1] = DualImage(cv::Range(0, DualImage.rows - 1), cv::Range(DualImage.cols/2, DualImage.cols- 1));
              cv::cvtColor(m_Frames[0], m_Frames[0], cv_conversion_code); 
              cv::cvtColor(m_Frames[1], m_Frames[1], cv_conversion_code); 
        }
        cv::Mat& GetLeft(){return m_Frames[0]; }
        cv::Mat& GetRight(){return m_Frames[1]; }
}; 