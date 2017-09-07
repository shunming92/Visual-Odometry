#pragma once

#ifndef OPTIMIZOR_H
#define OPTIMIZOR_H

#include <string>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

using namespace std;

namespace StereoVO
{
    class Optimizor
    {
    public:
        
        Optimizor();
        
        void poseOptimze(const vector<cv::Point3f> & point_3d, const vector<cv::Point2f>& point_2d, const cv::Mat& k, cv::Mat& R, cv::Mat& t);
    };
    
    
}


#endif
