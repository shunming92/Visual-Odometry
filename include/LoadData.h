#pragma once

#ifndef LOADDATA_H
#define LOADDATA_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std;

namespace StereoVO
{
    class LoadData
    {
    public:
        
        LoadData();
        ~LoadData();
        
        void loadImgData(const string& imgPathL, const string& imgPathR, const string& timePath);
        
        vector<cv::Mat> getImageDataL();
        vector<cv::Mat> getImageDataR();
        
        void test_visulization();       // test only
        
    private:
        
        void loadAddress(const string& imgPathL, const string& imgPathR, const string& timePath);
        
        vector<string> vecimgNameL, vecimgNameR;
        vector<cv::Mat> vecImageL, vecImageR;
    };
}

#endif
