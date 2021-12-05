#include "Stitcher.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
Stitcher::Stitcher(){
  m_SIFTDetector = cv::SIFT::create(); 

}
Stitcher::Stitcher(const std::shared_ptr<StereoCamera>& Camera){
    m_Camera = Camera; 
  m_WarpedImage.create(cv::Size(m_Camera->GetRawImage().cols, m_Camera->GetRawImage().rows), CV_8UC1); 
  m_SIFTDetector = cv::SIFT::create(); 
}
void Stitcher::FindFeaturePoints(){
  // find key points ; 
   m_SIFTDetector->detectAndCompute(m_Camera->GetLeftImage(),NULL , m_KeyPoints[0],m_Descriptors[0]); 
   m_SIFTDetector->detectAndCompute(m_Camera->GetRightImage(),NULL , m_KeyPoints[1],m_Descriptors[1]); 
}
void Stitcher::MatchFeaturePoints(){
  // find matched key points indices 
   cv::FlannBasedMatcher matcher; //https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
  std::vector<std::vector<cv::DMatch>> matched; 
  matcher.knnMatch(m_KeyPoints[0], m_KeyPoints[1], matched, 2); /// keep top 2 matches only

  std::vector<cv::DMatch> good_matches; 
      const float ratio_thresh = 0.7f;

    for (size_t i = 0; i < matched.size(); i++)
    {
        if (matched[i][0].distance < ratio_thresh * matched[i][1].distance)
        {
            good_matches.push_back(matched[i][0]);
        }
    }

  
  std::vector<cv::KeyPoint> left_keypoint; 
  std::vector<cv::KeyPoint> right_keypoint; 
  for(int i = 0; i < good_matches.size(); i++){
    left_keypoint.emplace_back(m_KeyPoints[0][good_matches[i].queryIdx]); 
    right_keypoint.emplace_back(m_KeyPoints[0][good_matches[i].trainIdx]); 
  }
  // calculate homography using RANSAC with an error thresholf of 5.0 
  m_Homography = cv::findHomography(left_keypoint, right_keypoint,cv::RANSAC, 5.0); 
}
void Stitcher::Stitch(const std::shared_ptr<StereoCamera>& Camera){
  m_Camera = Camera; 
  m_WarpedImage.create(cv::Size(m_Camera->GetRawImage().cols, m_Camera->GetRawImage().rows), CV_8UC1); 
  this->Stitch(); 
}
void Stitcher::Stitch(){

  if(m_Homography.empty()){
    FindFeaturePoints(); 
    MatchFeaturePoints(); 
  }
  // stitch images - will need optimization 
  cv::Mat& Image = m_Camera->GetRawImage(); 
  cv::warpPerspective(m_Camera->GetLeftImage(),m_WarpedImage,m_Homography ,m_WarpedImage.size()); 
  Image(cv::Range(0, Image.rows-1), cv::Range(0, Image.cols/2 - 1)) = m_WarpedImage(cv::Range(0, Image.rows-1), cv::Range(0, Image.cols/2 -1)); 
  // crop image and remove black borders 
 
  for (int j = (int)(Image.cols/2 -1); j>=0; j++){
     bool crop_here= true; 
    if(Image.at<uchar>(0, j) == 0){
      // found black pixel 
      // go through the rows 
      for(int i = 1; i < Image.rows ; i++){
          if(Image.at<uchar>(0, j) != 0){
            crop_here = false; 
            break; 
          }
          
      }
    }
    if(crop_here){
      m_Camera->GetStitchedImage() = Image(cv::Range(0,Image.rows-1), cv::Range(j, Image.cols-1)); 
      break; 
    }
  }
  
}


/*

 #include "opencv2/xfeatures2d.hpp"

  // 
  // now, you can no more create an instance on the 'stack', like in the tutorial
  // (yea, noticed for a fix/pr).
  // you will have to use cv::Ptr all the way down:
  //
  cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
  //cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
  //cv::Ptr<Feature2D> f2d = ORB::create();
  // you get the picture, i hope..

  //-- Step 1: Detect the keypoints:
  std::vector<KeyPoint> keypoints_1, keypoints_2;    
  f2d->detect( img_1, keypoints_1 );
  f2d->detect( img_2, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)    
  Mat descriptors_1, descriptors_2;    
  f2d->compute( img_1, keypoints_1, descriptors_1 );
  f2d->compute( img_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using BFMatcher :
  BFMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

*/ 

/*include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{        
  Mat image = imread("TestImage.jpg");

  // Create smart pointer for SIFT feature detector.
  Ptr<FeatureDetector> featureDetector = FeatureDetector::create("SIFT");
  vector<KeyPoint> keypoints;

  // Detect the keypoints
  featureDetector->detect(image, keypoints); // NOTE: featureDetector is a pointer hence the '->'.

  //Similarly, we create a smart pointer to the SIFT extractor.
  Ptr<DescriptorExtractor> featureExtractor = DescriptorExtractor::create("SIFT");

  // Compute the 128 dimension SIFT descriptor at each keypoint.
  // Each row in "descriptors" correspond to the SIFT descriptor for each keypoint
  Mat descriptors;
  featureExtractor->compute(image, keypoints, descriptors);

  // If you would like to draw the detected keypoint just to check
  Mat outputImage;
  Scalar keypointColor = Scalar(255, 0, 0);     // Blue keypoints.
  drawKeypoints(image, keypoints, outputImage, keypointColor, DrawMatchesFlags::DEFAULT);

  namedWindow("Output");
  imshow("Output", outputImage);

  char c = ' ';
  while ((c = waitKey(0)) != 'q');  // Keep window there until user presses 'q' to quit.

  return 0;

}*/ 
