#include "ConeClassifier.h"

ConeClassifier::ConeClassifier(){
    // load model 
   /* m_Net = cv::dnn::readNet(modelPath, configPath, parser.get<String>("framework"));
    int backend = 0;
    net.setPreferableBackend(backend);
    net.setPreferableTarget(parser.get<int>("target"));
    std::vector<String> outNames = net.getUnconnectedOutLayersNames();
    */
   
}
ConeClassifier::ConeClassifier(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
}
ConeClassifier::~ConeClassifier(){

}

void ConeClassifier::Detect(){
}

void ConeClassifier::Detect(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
    Detect(); 
}