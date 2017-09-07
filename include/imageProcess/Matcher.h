#pragma once

#ifndef MATCHER_H
#define MATCHER_H

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "basicStructure.h"
#include "track/Frame.h"


using namespace std;

namespace StereoVO
{
    const int FRAME_GRID_ROWS = 48;
    const int FRAME_GRID_COLS = 64;
    const int TH_HIGH = 100;
    const int TH_LOW = 50;
    const int HISTO_LENGTH = 30;
    
    class OrbExtractor;
    
    class Matcher
    {
    public:
        
        Matcher();
        Matcher(int mb_, int mbf_);
        ~Matcher();
        
        void matchProcess(int rows, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR);  // main function - ORBSLAM2 original
        
        void matchProcess_thread(int rows, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR);   // main function - SelfDefine
        
        void computeImageBounds(cv::Mat img_);
        
        void assignFeaturesToGrid(OrbExtractor* mpOrbExtractorL);
        
        int matcheTwoFrames(Frame& currFrame, Frame& lastFrame, float fx, float fy, float cx, float cy);
        
        vector<pair<cv::Point2f, cv::Point2f>> getMatch2D2D_norm();
        vector<pair<cv::Point2f, cv::Point2f>> getMatch2D2D_pixel();
        vector<int> getMatchedID();
        
        void test_visilization(OrbExtractor* mpOrbExtractorL, int rows, int cols, cv::Mat& imgL_, cv::Mat& imgR_);
        
        vector<size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]; 
        
        vector<float> muL, muR, mDepth;         // self define only
        
        
    private:
        
        void findBestMatchInCurrFrame(Frame& currFrame, Frame& lastFrame, vector<pair<cv::Point2f, cv::Point2f>>& matchedPoints, vector<int>& matchedID_last);
        
        void removeOutlinerRansac( vector<pair<cv::Point2f, cv::Point2f>>& vecMatched_original_, vector<int>& vecMatchedID_last_, float fx, float fy, float cx, float cy);
        
        int descriptorDistance(const cv::Mat& a, const cv::Mat& b);
        
        void stereoMatches(int threadnumber,int ID,int N, vector<vector<size_t>> &vRowIndices, float maxD, float minD, int thOrbDist, vector<pair<int, int> > &vDistIdx, vector<cv::KeyPoint>& kpsL, vector<cv::KeyPoint>& kpsR, cv::Mat& despL, cv::Mat despR, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR);
        
        bool posInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
        
        int mb, mbf;
        
        int mnMinX, mnMaxX, mnMinY, mnMaxY;     // self define only
        
        float mfGridElementWidthInv, mfGridElementHeightInv; // self define only
        
        vector<pair<cv::Point2f, cv::Point2f>> mvecMatch_2D_2D_norm;
        vector<pair<cv::Point2f, cv::Point2f>> mvecMatch_2D_2D_pixel;
        vector<int> mvecMatchedID;
    };
}

#endif

