#include "StereoCamera.h"
vn::StereoCamera::StereoCamera(){
    m_VisionDevice = new GXDevice(); 
    m_VisionDevice->Init(); 
}
vn::StereoCamera::StereoCamera(GXDevice* VisionDevice){
  
    m_VisionDevice = VisionDevice; 
      m_VisionDevice->StartStream(); 
}
vn::StereoCamera::~StereoCamera(){
    m_VisionDevice->StopStream(); 
    m_VisionDevice->Shutdown(); 
    delete m_VisionDevice ; 
}

void vn::StereoCamera::StartCapture() const{
    m_VisionDevice->StartStream(); 
}

void vn::StereoCamera::StopCapture() const{
    m_VisionDevice->StopStream(); 
}
void vn::StereoCamera::AquireImage(){
    PGX_FRAME_BUFFER p_FrameData = m_VisionDevice->AquireImage(); 
    
    if(p_FrameData->nStatus == GX_STATUS_SUCCESS){
        m_RawImage.create(p_FrameData->nHeight,p_FrameData->nWidth,CV_8UC1); //Creates if necessary otherwise it doesn't; //size = 2560 1024
        m_VideoImage.create(p_FrameData->nHeight,p_FrameData->nWidth,CV_8UC3);
       
                    
        m_RawImage.data = (uchar*)p_FrameData->pImgBuf; 
      cv::resize(m_RawImage, m_RawImage, cv::Size(1280, 512)); // 2 640 x 256
        cv::flip(m_RawImage,m_RawImage,0); 
       //  cv::cvtColor(m_RawImage, m_RawImage, cv::COLOR_BayerRG2GRAY);// don't need to convert to grayscale
       cv::cvtColor(m_RawImage,m_VideoImage,cv::COLOR_BayerRG2BGR); 
        
        
    }
}
 void vn::StereoCamera::SetImage(const cv::Mat& DualImage){
     m_RawImage = DualImage; 
 }
void vn::StereoCamera::SetImage(const cv::Mat& DualImage, int cv_conversion_code) {
    cv::cvtColor(DualImage, m_RawImage, cv_conversion_code); 
}
 
cv::Mat vn::StereoCamera::GetLeftImage(){
    return m_RawImage(cv::Range(0, m_RawImage.rows - 1), cv::Range(0, (m_RawImage.cols/2) - 1)); 
  
}
cv::Mat vn::StereoCamera::GetRightImage(){
    return m_RawImage(cv::Range(0, m_RawImage.rows - 1), cv::Range(m_RawImage.cols/2, m_RawImage.cols- 1)); 
    
}

cv::Mat& vn::StereoCamera::GetLeftIntrinsicParams(){
    return m_IntrinsicParams[0];
}
cv::Mat& vn::StereoCamera::GetRigthIntrinsicParams(){
    return m_IntrinsicParams[1];
}
cv::Mat& vn::StereoCamera::GetLeftDistortionCoeff(){
    return m_DistortionCoeff[0]; 
} 
cv::Mat& vn::StereoCamera::GetRightDistortionCoeff(){
    return m_DistortionCoeff[1]; 
}
cv::Mat& vn::StereoCamera::GetRawImage(){return m_RawImage; } 
cv::Mat& vn::StereoCamera::GetVideoImage(){return m_VideoImage; }
cv::Mat& vn::StereoCamera::GetStitchedImage(){return m_StitchedImage; }
