#pragma once

#ifndef FRAME_H
#define FRAME_H

#include <set>
#include <map>
#include <list>

#include "../basicStructure.h"

using namespace std;

namespace StereoVO
{
    class MapPoint;
    
    class Frame
    {
    public:
        
        Frame();
        Frame(featureInfo& featureMsg, int farThreshold_);
        
        //copy constructor
        Frame(const Frame &frame);

        ~Frame();
        
        void setPose();
        
        void getCameraParameter(float fx_, float fy_, float cx_, float cy_, float mbf_, float mb_, int farThreshold_, cv::Mat& k_);
        
        cv::Mat getImg();
        cv::Mat getDesp();
        vector<cv::Point2f> getKps_camNorm();
        vector<float> getDepthes();
        
        cv::Mat getRcw();
        cv::Mat getRwc();
        cv::Mat gettwc();
        cv::Mat gettcw();
        
        void setRotation(cv::Mat& R);
        void setTranslation(cv::Mat& t);

        
    private:
        
        vector<cv::Point2f> vecKps_camNorm;
        
        vector<float> vecDepthes;
        
        cv::Mat mdesp;
        
        cv::Mat mimg;    // for test purpose
        
        cv::Mat mTcw, mTwc, mRcw, mRwc, mtcw, mtwc;
        
        float fx, fy, cx, cy, invfx, invfy;
        
        float mb, mbf, farThreshold;
        
        cv::Mat k;
        bool print;

    };
}

#endif
