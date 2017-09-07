#pragma once

#ifndef RECTIFIER_H
#define RECTIFIER_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

namespace StereoVO
{
    class Rectifier
    {
    public:
        
        Rectifier(const string& configPath, const cv::Mat& k_, const cv::Mat DistCoef_);
        ~Rectifier();
        
        void rectImage(const cv::Mat& imgL_, const cv::Mat& imgR_, int flag);    // for stereo, rect entire img
        
        void test_visualizeRectStereo(cv::Mat& imgL_, cv::Mat& imgR_); // optional
        
        cv::Mat stereoRectL, stereoRectR;   // rected stereo mat
        
        cv::Mat K_l, K_r, P_l, P_r;         // stereo camera parameters
        cv::Mat R_l, R_r, D_l, D_r;         // stereo camera parameters
        
    private:
        
        cv::Mat rectEntireImg(const cv::Mat& img_, const cv::Mat& K_, const cv::Mat& D_, const cv::Mat& R_, const cv::Mat& P_);
        
        void undistortKeyPoints(vector<cv::KeyPoint>& keyPoints);
        
        int rows_l, rows_r, cols_l, cols_r; // stereo camera parameters
        
        cv::Mat mk, mDistCoef;        // mono camera parameters
    };
    
}



#endif
