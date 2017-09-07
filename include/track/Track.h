#pragma once

#ifndef TRACK_H
#define TRACK_H

#include <queue>
#include <mutex>
#include <condition_variable>

#include "../basicStructure.h"
#include "System.h"
#include "track/Frame.h"
#include "track/optimizor.h"

using namespace std;

namespace StereoVO
{
    class System;
    
    class Frame;
    
    class Matcher;
    
    class Track
    {
    public:
        
        Track();
        ~Track();
        
        void readParameters(const string& configPath);
        
        void process();
        
        void setFeatureBuff(featureInfo& info);
        
        void setStopFlag(bool flag);
        
        static bool checkFeatureBuffnotEmpty();
        static queue<featureInfo> mFeatureBuff;
        
    private:
        
        Matcher* mpMatcher;
        
        void init();
        
        void creatCurrFrame();
        
        cv::Point3f pointsCamnorm2World(cv::Point2f& point_norm_, float z_, int i);
        
        void points3DCam2World(vector<cv::Point3f>& mapPoints_w, vector<pair<cv::Point3f, cv::Point2f>>& matches_3D_2D);
        
        void pointsPixel2Cam(vector<pair<cv::Point3f, cv::Point2f>>& matches_3D_2D, vector<cv::Point3f>& mapPoints_c_norm_curr, vector<cv::Point2f>& pixelPoints_curr);
        // void creatLastandCurrFrame(bool& flag);
        
        void computePose();
        
        void solvePnpProblem(vector<cv::Point3f>& points3D, vector<cv::Point2f>& points2D, cv::Mat& R, cv::Mat& t);
        
        void getMapPointsandPixelPoints(vector<pair<Eigen::Vector3f, cv::Point2f>>& matches, vector<cv::Point3f> mapPoints, vector<cv::Point2f> pixelPoints, int i);
        
        void getMapPointsandPixelPoints(vector<pair<Eigen::Vector3f, cv::Point2f>>& matches, vector<cv::Point3f> mapPoints, vector<cv::Point2f> pixelPoints);
        
        void test_visulFrameMatchResult(cv::Mat& left, cv::Mat& right);
        
        imgInfo getBuff();
        
        enum trackProcessStatus { Init = 1, Norm = 2 };
        trackProcessStatus mTrackStatus;
        
        Frame mCurrFrame;
        Frame mLastFrame;
        
        Optimizor poseOptimizor;
        // System* mpSystem;
        
        float fx, fy, cx, cy;     
        float k1, k2, p1, p2; 
        float farThreshold, mb, mbf;
        
        mutex mMutexBuff;
        mutex mMutexFeatureBuff;
        mutex mMutexStop;
        
        cv::Mat mDistCoef, mk;
        
        bool stop_flag;
        bool buffEmpty_flag;
        
        condition_variable con_featureBuff;
        
        Eigen::Matrix4d traject_pose;
        Eigen::Vector3d position1;
        vector<cv::Mat> savedTrajactory;
    };
}

#endif
