#include "LoadData.h"

namespace StereoVO
{
    LoadData::LoadData()
    {
    }
    
    LoadData::~LoadData()
    {
    }
    
    void LoadData::loadImgData(const string& imgPathL, const string& imgPathR, const string& timePath)
    {
        loadAddress(imgPathL, imgPathR, timePath);
        
        vecImageL.reserve(2500);
        vecImageR.reserve(2500);
        for (int i = 0; i < vecimgNameL.size(); i++)
        {
            cv::Mat imgL = cv::imread(vecimgNameL[i], CV_LOAD_IMAGE_UNCHANGED);
            cv::Mat imgR = cv::imread(vecimgNameR[i], CV_LOAD_IMAGE_UNCHANGED);
            vecImageL.push_back(imgL);
            vecImageR.push_back(imgR);
        }
        
    }
    
    void LoadData::loadAddress(const string& imgPathL, const string& imgPathR, const string& timePath)
    {
        ifstream fTimes;
        fTimes.open(timePath.c_str());
        
        vecimgNameL.reserve(2500);
        vecimgNameR.reserve(2500);
        
        while(!fTimes.eof())      
        {
            string s;
            getline(fTimes,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                vecimgNameL.push_back(imgPathL + "/" + ss.str() + ".png");
                vecimgNameR.push_back(imgPathR + "/" + ss.str() + ".png");
            }
        }
    }
    
    vector<cv::Mat> LoadData::getImageDataL()
    {
        return vecImageL;
    }
    
    vector<cv::Mat> LoadData::getImageDataR()
    {
        return vecImageR;
    }
    
    void LoadData::test_visulization()
    {
        for (int i = 0; i < vecImageL.size(); i++)
        {
            cv::imshow("Load data - Left", vecImageL[i]);
            cv::imshow("Load data - Right", vecImageR[i]);
            cv::waitKey(20);
        }
    }
}
