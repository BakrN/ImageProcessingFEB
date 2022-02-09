#pragma once 
#include "StereoCamera.h"


#define PYTORCH_MODEL  
#define CONFIDENCE_THRESHOLD 0.9
namespace vn {
struct NetConfiguration{
    std::string modelpath; 
    std::string configpath; 
    std::string framework; 
    int backend; 
}; 
typedef struct { 
   cv::Rect2f bounding_box; 
   char color ; // indicates left or right cone 
} ConeDetection; 
class ConeClassifier{
    private: 
    NetConfiguration m_NetConfiguration;
    std::shared_ptr<StereoCamera> m_Camera; 
    
    cv::dnn::Net m_Net; 
    std::vector<ConeDetection> m_ConeDetections; 
    public: 
    ConeClassifier(); 
    ConeClassifier(const std::shared_ptr<StereoCamera>& Camera); 
    ~ConeClassifier(); 

    void Detect(); 
    void Detect(const std::shared_ptr<StereoCamera>& Camera); 

    std::vector<ConeDetection>& GetDetections() ; 
    std::vector<ConeDetection>::iterator begin(){return m_ConeDetections.begin(); }
    std::vector<ConeDetection>::iterator end(){return m_ConeDetections.end(); }


}; } ; 