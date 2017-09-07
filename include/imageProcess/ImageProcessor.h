#pragma once

#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <queue>
#include <fstream>
#include <thread>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "basicStructure.h"

using namespace std;

namespace StereoVO
{
    class Rectifier;
    class OrbExtractor;
    class Matcher;
    
    class ImageProcessor
    {
    public:
        
        ImageProcessor(const string& configPath);
        ~ImageProcessor();
        
        featureInfo process(cv::Mat& imgL_, cv::Mat& imgR_);
        
        bool frameQualityFlag;  // check current frame is good or bad
        
    private:
        
        void rectStereoImg(cv::Mat& imgL_, cv::Mat& imgR_);
        
        // 0 one trhead, 1 multi-thread
        void featureDetect(cv::Mat& imgL_, cv::Mat& imgR_, int flag);
        
        // 0 one trhead, 1 multi-thread
        void featureMatch(cv::Mat& imgL_, cv::Mat& imgR_,int flag);
        
        void assigntoGrid(cv::Mat& imgL_);
        
        vector<cv::Point2f> projectIntoCamNorm();
        
        void constructQueforThread();
        
        Rectifier* mpRectifierL;
        Rectifier* mpRectifierR;
        
        OrbExtractor* mpOrbExtractorL;
        OrbExtractor* mpOrbExtractorR;
        
        Matcher* mpMatcher;
        
        // vector<keypoint_SelfDefine> mvecKps_SelfDefine;
        
        int featureTrackTimes;                  // record process time of this program
        
        cv::Mat mDistCoef, mk;                  // camera parameters
        float fx, fy, cx, cy, rows, cols;       // camera parameters
        float k1, k2, p1, p2;                   // camera parameters
        float fps, mbf, farThreshold, mb;       // camera parameters
        
        float scaleFactor;                                      // orb parameters
        int num, nLevels, iniThFAST, minThFAST, edgeThreshold;  // orb parameters
        int firstLevel, WTA_K, patchSize, fastThreshold;        // orb parameters
        int miniNum, miniMatch, miniNumMono;                    // orb parameters
        
        vector<cv::KeyPoint> mStereoKpL, mStereoKpR;

        cv::Mat mStereoDespL, mStereoDespR;
        
        featureInfo mFeatureInfo;
    };
}

#endif
