#pragma once

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <string>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;

namespace StereoVO
{
    class ExtractorNode
    {
    public:
        ExtractorNode():bNoMore(false){}
        
        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);
        
        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };
    
    class OrbExtractor
    {
    public:
        
        OrbExtractor(int maxNum_, float scaleFactor_, int nLevels_, int iniThFAST_, int minThFAST_);
        ~OrbExtractor();
        
        void extractProcess(const cv::Mat& img_);       // main function - ORBSLAM2 original
        
        void extractProcess_thread(const cv::Mat img_);  // main function - SelfDefine

        void test_DrawMatches(cv::Mat& img_);           // only in test
        
        vector<cv::KeyPoint> getOrbKeyPoints();
        cv::Mat getOrbDesp();
        
        vector<cv::Mat> mvImagePyramid;                 // only orbExtract_SelfDefine
        vector<int> mnFeaturesPerLevel, uman;           // only orbExtract_SelfDefine
        vector<float> mvScaleFactor, mvInvScaleFactor;  // only orbExtract_SelfDefine
        vector<float> mvLevelSigma2, mvInvLevelSigma2;  // only orbExtract_SelfDefine
        vector<cv::Point> pattern;                      // only orbExtract_SelfDefine
        vector<int> umax;                               // only orbExtract_SelfDefine
        int iniThFAST, minThFAST;                       // only orbExtract_SelfDefine
        
    private:
        
        void processPreparation();
        
        void computePyramid(cv::Mat& img_);
        
        void computeKeyPointsOctTree(vector<vector<cv::KeyPoint> >& allKeypoints);
        
        void computeDesp(vector < vector<cv::KeyPoint> >& allKeypoints, vector<cv::KeyPoint>& kpsTemp, cv::Mat& despTemp);
        
        void computeDescriptors(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,const vector<cv::Point>& pattern);
        
        void computeOrbDescriptor(const cv::KeyPoint& kpt,const cv::Mat& img, const cv::Point* pattern,uchar* desc);
        
        void computeOrientation(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, const vector<int>& umax);
        
        float IC_Angle(const cv::Mat& image, cv::Point2f pt,  const vector<int> & u_max);
        
        vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);

        
        void computeKeyPointsOctTree_ParalleThread(vector<vector<cv::KeyPoint> >& allKeypoints);
        
        void computeKeyPointsOctTreeEveryLevel(int nlevel, int ID,vector< vector< cv::KeyPoint > >& allKeypoints);
        
        void computeOrientation_ParalleThread(int nleves, int threadnumber, int ID, vector<cv::Mat>& mvImagePyramid, vector<vector<cv::KeyPoint> >& allKeypoints, vector<int>& umax);
        
        void computeDesp_ParalleThread(vector < vector<cv::KeyPoint> >& allKeypoints, vector<cv::KeyPoint>& kpsTemp, cv::Mat& despTemp);
        
        void computeDescriptorsEveryLevel(int level, vector<vector<cv::KeyPoint> > &allKeypoints, const cv::Mat& descriptors, int offset, vector<cv::KeyPoint>& _keypoints, int i, cv::Mat& desc_temp_mat);
        
        
        vector<cv::KeyPoint> mKeyPoints;
        
        cv::Mat mDesp;
        
        int maxNum, nLevels;
        float scaleFactor;
        
        int test_counter;                               // only in test
        cv::Mat test_imgLast, test_imgCurrent;          // only in test
        vector<cv::KeyPoint> test_kpsLast, test_kpsCurrent; // only in test
        cv::Mat test_despLast, test_despCurrent;        // only in test
        
    };
}
#endif
