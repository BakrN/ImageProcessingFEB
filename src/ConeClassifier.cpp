#include "ConeClassifier.h"

vn::ConeClassifier::ConeClassifier(){
 

}
vn::ConeClassifier::ConeClassifier(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
}
vn::ConeClassifier::~ConeClassifier(){

}

void vn::ConeClassifier::Detect(){
        //preprocess image to do undermine illumination effects 
    cv::Mat pre_image; 
    //process output 
    cv::Mat detections;
    // a mat with rows of all detections 
    for (int i = 0 ; i < detections.rows; i++){ 
        if(detections.at<float>(i , 1)>= CONFIDENCE_THRESHOLD){ // confidence 
            // add bounding box to vector 

            // calculate color by doing some image processing 
        }
    }

}

void vn::ConeClassifier::Detect(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
    Detect(); 
}

// simple canny edge detector with threshhold 
cv::Mat& EdgeDetect(const cv::Mat& ConeImage){ 
    cv::Mat edge_image; 
    edge_image.create(ConeImage.size(), CV_8UC1); 

    return edge_image; 
}
// Hough transform 
// look for line within range 