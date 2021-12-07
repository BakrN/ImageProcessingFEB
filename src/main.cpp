

#include <ros/ros.h> 
#include "GXDevice.h"
#include "StereoCamera.h"
#include "calibration/Calibrator.h"
#include "stitching/Stitcher.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>






int main(int argc, char** argv){
    ros::init(argc, argv,"vision_node"); 
    ros::NodeHandle nh; 
    GXDevice VisionDevice; 
     VisionDevice.Init(); 
    std::shared_ptr<StereoCamera> Camera = std::make_shared<StereoCamera>(&VisionDevice); 
    Calibrator calibrator(Camera); 
    Stitcher stitcher(Camera); 
   

    
    //calibrator.Calibrate(0); 
    
    /*while(1) {
        Camera->AquireImage(); 
        cv::imshow("window", Camera->GetVideoImage()); 
        if(cv::waitKey(1) == (int)('q')){
            break; 
        }
    }*/
    cv::namedWindow("StitchedImage"); 
    cv::namedWindow("LeftImage"); 
    cv::namedWindow("RightImage"); 
    while(1) {
        Camera->AquireImage(); 
        stitcher.Stitch(); 
        cv::imshow("StitchedImage", Camera->GetStitchedImage()); 
        cv::imshow("LeftImage", Camera->GetLeftImage()); 
        cv::imshow("RightImage", Camera->GetRightImage()); 
        if(cv::waitKey(1) == (int)('q')){
            break; 
        }
    }

    ROS_INFO("STOPPED STREAM"); 
    

    ros::shutdown(); 
    return 0; 
}