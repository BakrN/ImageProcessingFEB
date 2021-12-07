#pragma once 
#include "StereoCamera.h"
#include <opencv2/dnn.hpp>
struct NetConfiguration{
    std::string modelpath; 
    std::string configpath; 
    std::string framework; 
    int backend; 
}; 
class ConeClassifier{
    private: 
    std::shared_ptr<StereoCamera> m_Camera; 
    cv::dnn::Net m_Net; 
    
    public: 
    ConeClassifier(); 
    ConeClassifier(const std::shared_ptr<StereoCamera>& Camera); 
    ~ConeClassifier(); 

    void Detect(); 
    void Detect(const std::shared_ptr<StereoCamera>& Camera); 


}; 