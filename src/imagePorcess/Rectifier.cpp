#include "imageProcess/Rectifier.h"

using namespace std;

namespace StereoVO
{
    Rectifier::Rectifier(const string& configPath, const cv::Mat& k_, const cv::Mat DistCoef_):
            mk(k_), mDistCoef(DistCoef_)
    {
        cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
        }

        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        rows_l = fsSettings["LEFT.height"];
        cols_l = fsSettings["LEFT.width"];
        rows_r = fsSettings["RIGHT.height"];
        cols_r = fsSettings["RIGHT.width"];
        
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "Error: Calibration parameters to rectify stereo are missing!" << endl;
            exit(-1);
        }
    }
    
    Rectifier::~Rectifier()
    {
    }
    
    void Rectifier::rectImage(const cv::Mat& imgL_, const cv::Mat& imgR_, int flag)
    {
        if(flag == 0)
            stereoRectL = rectEntireImg(imgL_, K_l, D_l, R_l, P_l);
        else if (flag == 1)
            stereoRectR = rectEntireImg(imgR_, K_r, D_r, R_r, P_r);
    }
    
    cv::Mat Rectifier::rectEntireImg(const cv::Mat& img_, const cv::Mat& K_, const cv::Mat& D_, const cv::Mat& R_, const cv::Mat& P_)
    {
        cv::Mat M1, M2;
        cv::initUndistortRectifyMap(K_,D_,R_,P_.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1,M2);
        
        cv::Mat rectImgTemp;
        cv::remap(img_,rectImgTemp,M1,M2,cv::INTER_LINEAR);
        return rectImgTemp;
    }
    
    void Rectifier:: test_visualizeRectStereo(cv::Mat& imgL_, cv::Mat& imgR_)
    {
        cv::Mat imgRectL = imgL_.clone();
        cv::Mat imgRectR = imgR_.clone();
        
        int imgHeight = rows_l;
        int imgWidth = cols_l;
        
        // draw rect
        cv::Mat showImageRect(imgHeight,2*imgWidth,imgRectL.type());
        cv::Rect LeftRect(0,0,imgWidth,imgHeight);
        cv::Rect RightRect(imgWidth,0,imgWidth,imgHeight);
        imgRectL.copyTo(showImageRect(LeftRect));
        imgRectR.copyTo(showImageRect(RightRect));
        
        // draw epipolar line
        for( int i = 0; i < imgHeight; i += 32 )
        {
            line(showImageRect, cv::Point(0, i), cv::Point(2*imgWidth, i), cv::Scalar(0, 255, 0), 1, 8);
        }
        
        // visualize 
        cv::imshow("rect", showImageRect);
        cv::waitKey(20);
    }

    
}
