#include "../src/stitching/Stitcher.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem> 


void RunStitchTest(){
    std::string Dir = std::filesystem::current_path().string(); 
    
    //cv::namedWindow("Left"); 
    //cv::namedWindow("Right"); 
    //cv::namedWindow("Stitched"); 
    Stitcher stitcher; 
    
    for(int i = 1; i < 21; i++){
    cv::Mat left = cv::imread(Dir+"/CalibrationImages/LEFT"+std::to_string(i)+".jpg", cv::IMREAD_GRAYSCALE); 
    cv::Mat right= cv::imread(Dir+"/CalibrationImages/RIGHT"+std::to_string(i)+".jpg", cv::IMREAD_GRAYSCALE); 
    cv::Mat out = stitcher.Stitch(left, right); 

    //cv::imshow("Left", left); 
    //cv::imshow("Right", right); 
    //cv::imshow("Stitched", out); 
    //cv::waitKey(0); 
    }

    //cv::destroyAllWindows(); 


}