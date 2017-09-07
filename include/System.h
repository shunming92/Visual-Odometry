#pragma once

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "basicStructure.h"

using namespace std;

namespace StereoVO
{
    class ImageProcessor;
    class Track;
    
    class System
    {
    public:
        
        System(const string& configPath);
        ~System();
        
        void processStereo(cv::Mat& imgL, cv::Mat& imgR);
        
        void setStopFlag();
        
    private:
        
        void setMsgtoTrack(featureInfo& feature_info_);
        
        ImageProcessor* mpImageProcessor;
        Track* mpTrack;
        
        thread* mptTrack;
        
        bool imageProcessorStop;
    };
}


#endif
