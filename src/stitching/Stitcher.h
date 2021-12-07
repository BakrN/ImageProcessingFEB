#pragma once
#include "../StereoCamera.h"
#include <vector>
#include <opencv2/features2d.hpp>
class Stitcher
{
private:
    std::shared_ptr<StereoCamera> m_Camera;
    
    cv::Ptr<cv::SIFT> m_SIFTDetector;
    std::vector<cv::KeyPoint> m_KeyPoints[2]; // left and right
    cv::Mat m_Descriptors[2]; 
    cv::Mat m_Homography; // From left to right
    cv::Mat m_WarpedImage; 
    void FindFeaturePoints();
    void MatchFeaturePoints();

public:
    Stitcher();
    Stitcher(const std::shared_ptr<StereoCamera> &Camera);
    ~Stitcher() = default;

    void Stitch(const std::shared_ptr<StereoCamera>& Camera); // updates the m_Image to one image
    void Stitch();
    cv::Mat &GetHomograhy();
};