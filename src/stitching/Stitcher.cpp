#include "Stitcher.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

vn::Stitcher::Stitcher(){
  m_SIFTDetector = cv::SIFT::create(); 
}
vn::Stitcher::Stitcher(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
  
  m_SIFTDetector = cv::SIFT::create(); 
}
void vn::Stitcher::FindFeaturePoints(){
  // find key points ; 

   m_SIFTDetector->detectAndCompute(m_Camera->GetLeftImage(),cv::noArray() , m_KeyPoints[0],m_Descriptors[0]); 
   m_SIFTDetector->detectAndCompute(m_Camera->GetRightImage(),cv::noArray() , m_KeyPoints[1],m_Descriptors[1]); 
}

void vn::Stitcher::FindFeaturePoints(const cv::Mat& Left, const cv::Mat& Right){
    m_SIFTDetector->detectAndCompute(Left,cv::noArray() , m_KeyPoints[0],m_Descriptors[0]); 
   m_SIFTDetector->detectAndCompute(Right,cv::noArray() , m_KeyPoints[1],m_Descriptors[1]); 
}
void vn::Stitcher::MatchFeaturePoints(){
  // find matched key points indices 
   cv::FlannBasedMatcher matcher; //https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
  std::vector<std::vector<cv::DMatch>> matched; 
  matcher.knnMatch(m_Descriptors[0], m_Descriptors[1], matched, 2); /// keep top 2 matches only

  std::vector<cv::DMatch> good_matches; 
      const float ratio_thresh = 0.7f;

    for (size_t i = 0; i < matched.size(); i++)
    {
        if (matched[i][0].distance < ratio_thresh * matched[i][1].distance)
        {
            good_matches.push_back(matched[i][0]);
        }
    }

  std::vector<cv::Point2f> left_keypoint; 
  std::vector<cv::Point2f> right_keypoint; 
  for(int i = 0; i < good_matches.size(); i++){
    left_keypoint.push_back(m_KeyPoints[0][good_matches[i].queryIdx].pt); 
    right_keypoint.push_back(m_KeyPoints[1][good_matches[i].trainIdx].pt); 
  }
  // calculate homography using RANSAC with an error thresholf of 5.0 
  m_Homography = cv::findHomography(right_keypoint, left_keypoint,cv::RANSAC, 3.0); // right to left
}
void vn::Stitcher::Stitch(const std::shared_ptr<StereoCamera>& Camera){
  m_Camera = Camera; 
  m_WarpedImage.create(cv::Size(m_Camera->GetRawImage().cols, m_Camera->GetRawImage().rows), CV_8UC1); 
  m_Homography.release(); // Assumes new camera; // reset 
  this->Stitch(); 
}
void vn::Stitcher::Stitch(){

  if(m_Homography.empty()){
   
    ROS_INFO("BEFORE MEAURING FEATURE POINTS"); 
    FindFeaturePoints(); 
    ROS_INFO("after MEAURING FEATURE POINTS"); 
    MatchFeaturePoints(); 

    ROS_INFO("After matching FEATURE POINTS"); 

  }
  // stitch images - will need optimization 
  cv::Mat& Image = m_Camera->GetRawImage(); 
  m_WarpedImage.create(Image.rows, Image.cols/2, CV_8UC1); 
    
  m_Camera->GetStitchedImage().create(Image.rows,Image.cols,CV_8UC1); 

  cv::warpPerspective(m_Camera->GetRightImage(),m_WarpedImage,m_Homography ,m_WarpedImage.size()); 
  m_WarpedImage.copyTo(m_Camera->GetStitchedImage()(cv::Range(0,Image.rows-1), cv::Range(Image.cols/2, Image.cols-1)) );
  m_Camera->GetLeftImage().copyTo( m_Camera->GetStitchedImage()(cv::Range(0,Image.rows-1), cv::Range(0,Image.cols/2 -1)));  
}

 cv::Mat vn::Stitcher::Stitch(const cv::Mat& Left, const cv::Mat& Right){
  if (m_Homography.empty()){
   
    FindFeaturePoints(Left, Right);   
    ROS_INFO("FOUND FEATURES  FEATURES"); 
    MatchFeaturePoints(); 
     ROS_INFO("FOUND MATCHED FEATURES"); }
  cv::Mat stitched_image; 
  cv::warpPerspective(Right, m_WarpedImage, m_Homography ,cv::Size(Right.cols+Left.cols, Right.rows)); 
  Left.copyTo(m_WarpedImage(cv::Rect(0,0,Left.cols, Left.rows))); 
  return m_WarpedImage; 
}

