

#include <ros/ros.h> 
#include "GXDevice.h"
#include "StereoCamera.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>






int main(int argc, char** argv){
    ros::init(argc, argv,"vision_node"); 
    ros::NodeHandle nh; 
    StereoCamera Camera; 
    GXDevice VisionDevice; 
    VisionDevice.Init(); 
      

    cv::namedWindow("LeftFeed"); 
    cv::namedWindow("RightFeed");
    cv::Mat Data; 
    VisionDevice.StartStream(); 
    while(1){
        PGX_FRAME_BUFFER p_FrameData= VisionDevice.AquireImage();
        if(!p_FrameData){
            ROS_INFO("NULL Exit"); 
            break; 
        }
        if(p_FrameData->nStatus == GX_STATUS_SUCCESS){
       
        //Do some image processing operations.
         //cv::Size(p_FrameData->nHeight, p_FrameData->nWidth),CV_8UC1,(void*)(p_FrameData->pImgBuf)); 
        Data.create(p_FrameData->nHeight,p_FrameData->nWidth,CV_8UC1); 
        Data.data = (uchar*)p_FrameData->pImgBuf; 
        //cv::cvtColor(Data,Data,cv::COLOR_BayerRG2GRAY); 
        cv::flip(Data,Data,0); 
        //imshow("LeftFeed", Data); 
        //ROS_INFO("DATA CREATED"); 
        //Data.data = (uchar*) p_FrameData->pImgBuf; //cpy ctr 
        // ROS_INFO("DATA ASSIGNED"); 
        
        Camera.SetImage(Data, cv::COLOR_BayerRG2GRAY); 
        cv::Mat& LeftImg =  Camera.GetLeftImage(); 
        cv::Mat& RightImg =  Camera.GetRightImage(); 
        cv::resize(LeftImg,LeftImg, cv::Size(LeftImg.rows/2, LeftImg.cols/2)); 
        cv::resize(RightImg,RightImg, cv::Size(RightImg.rows/2, RightImg.cols/2)); 
        imshow("LeftFeed", LeftImg); 
        imshow("RightFeed", RightImg); 
  
        }
        if(cv::waitKey(1) == (int)('q')){
            break; 
        } 
    } 
     ROS_INFO("STOPPED STREAM"); 
    VisionDevice.StopStream(); 
    cv::destroyAllWindows(); 

    VisionDevice.Shutdown(); 
    ros::shutdown(); 
    return 0; 
}