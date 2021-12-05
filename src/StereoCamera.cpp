#include "StereoCamera.h"

StereoCamera::StereoCamera(){
    m_VisionDevice = new GXDevice(); 
    m_VisionDevice->Init(); 
}
StereoCamera::StereoCamera(GXDevice* VisionDevice){
    m_VisionDevice = VisionDevice; 
}
StereoCamera::~StereoCamera(){
    m_VisionDevice->StopStream(); 
    m_VisionDevice->Shutdown(); 
    delete m_VisionDevice ; 
}

void StereoCamera::StartCapture() const{
    m_VisionDevice->StartStream(); 
}

void StereoCamera::StopCapture() const{
    m_VisionDevice->StopStream(); 
}
void StereoCamera::AquireImage(){
    PGX_FRAME_BUFFER p_FrameData = m_VisionDevice->AquireImage(); 
    if(p_FrameData->nStatus == GX_STATUS_SUCCESS){
        m_RawImage.create(p_FrameData->nHeight,p_FrameData->nWidth,CV_8UC1); //Creates if necessary otherwise it doesn't; 
        m_RawImage.data = (uchar*)p_FrameData->pImgBuf; 
        cv::flip(m_RawImage, m_RawImage,0); 
        //color cvt
        cv::cvtColor(m_RawImage, m_RawImage, cv::COLOR_BayerRG2GRAY); 
        //Resize
        cv::resize(m_RawImage ,m_RawImage, cv::Size(m_RawImage.rows/2, m_RawImage.cols/2)); // halving image size 
        
    }
}
 void StereoCamera::SetImage(const cv::Mat& DualImage){
     m_RawImage = DualImage; 
 }
void StereoCamera::SetImage(const cv::Mat& DualImage, int cv_conversion_code) {
    cv::cvtColor(DualImage, m_RawImage, cv_conversion_code); 
}
 
cv::Mat& StereoCamera::GetLeftImage(){
    cv::Mat Left= m_RawImage(cv::Range(0, m_RawImage.rows - 1), cv::Range(0, (m_RawImage.cols/2) - 1)); 
    return Left; 
}
cv::Mat& StereoCamera::GetRightImage(){
    cv::Mat Right= m_RawImage(cv::Range(0, m_RawImage.rows - 1), cv::Range(m_RawImage.cols/2, m_RawImage.cols- 1)); 
    return Right; 
}

cv::Mat& StereoCamera::GetLeftIntrinsicParams(){
    return m_IntrinsicParams[0];
}
cv::Mat& StereoCamera::GetRigthIntrinsicParams(){
    return m_IntrinsicParams[1];
}
cv::Mat& StereoCamera::GetLeftDistortionCoeff(){
    return m_DistortionCoeff[0]; 
} 
cv::Mat& StereoCamera::GetRightDistortionCoeff(){
    return m_DistortionCoeff[1]; 
}
cv::Mat& StereoCamera::GetRawImage(){return m_RawImage; } 
cv::Mat& StereoCamera::GetStitchedImage(){return m_StitchedImage; }
