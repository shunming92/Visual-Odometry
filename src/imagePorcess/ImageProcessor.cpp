#include "imageProcess/Rectifier.h"
#include "imageProcess/OrbExtractor.h"
#include "imageProcess/Matcher.h"
#include "timer/tictoc.h"

#include "imageProcess/ImageProcessor.h"

using namespace std;

namespace StereoVO
{
    ImageProcessor::ImageProcessor(const string& configPath)
    {
        cv::FileStorage fsSettings(configPath.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "--- Error: failed to open setting files at: "<< configPath<<endl;
            exit(-1);
        }
        
        // load camera && orb extractor parameters
        fx = fsSettings["Camera.fx"];
        fy = fsSettings["Camera.fy"];
        cx = fsSettings["Camera.cx"];
        cy = fsSettings["Camera.cy"];
        
        k1 = fsSettings["Camera.k1"];
        k2 = fsSettings["Camera.k2"];
        p1 = fsSettings["Camera.p1"];
        p2 = fsSettings["Camera.p2"];
        
        rows = fsSettings["Camera.height"];
        cols = fsSettings["Camera.width"];
        
        fps = fsSettings["Camera.fps"];
        
        mbf = fsSettings["Camera.bf"];    // baseline time fx
        mb = mbf/fx;
        
        num = fsSettings["ORBextractor.nFeatures"];
        scaleFactor = fsSettings["ORBextractor.scaleFactor"];
        nLevels = fsSettings["ORBextractor.nLevels"];
        iniThFAST = fsSettings["ORBextractor.iniThFAST"];
        minThFAST = fsSettings["ORBextractor.minThFAST"];
        edgeThreshold = fsSettings["ORBextractor.edgeThreshold"];
        firstLevel = fsSettings["ORBextractor.firstLevel"];
        WTA_K = fsSettings["ORBextractor.WTA_K"];
        patchSize = fsSettings["ORBextractor.patchSize"];
        fastThreshold = fsSettings["ORBextractor.fastThreshold"];
        miniNum = fsSettings["ORBextractor.miniNum"];
        miniMatch = fsSettings["ORBextractor.miniMatch"];
        miniNumMono = fsSettings["ORBextractor.miniNumMono"];
        int threshold = fsSettings["ThDepth"];
        
        farThreshold = mb *threshold; 

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = k1;
        DistCoef.at<float>(1) = k2;
        DistCoef.at<float>(2) = p1;
        DistCoef.at<float>(3) = p2;
        DistCoef.copyTo(mDistCoef);
        
        cv::Mat k = cv::Mat::eye(3, 3, CV_32F);
        k.at<float>(0,0) = fx;
        k.at<float>(0,2) = cx;
        k.at<float>(1,1) = fy;
        k.at<float>(1,2) = cy;
        k.copyTo(mk);
        
        featureTrackTimes = 0;
        
        cout << "--- Parameters: "<< endl;
        cout << "fps: "<< fps<<endl;
        cout << "mbf: "<< mbf<<endl;
        cout << "feature number: "<< num<<endl;
        cout << "scaleFactor: "<< scaleFactor<<endl;
        cout << "nLevels: "<< nLevels<<endl;
        cout << "iniThFAST: "<< iniThFAST<<endl;
        cout << "minThFAST: "<< minThFAST<<endl;
        cout << "edgeThreshold: "<< edgeThreshold<<endl;
        cout << "firstLevel: "<< firstLevel<<endl;
        cout << "WTA_K: "<< WTA_K<<endl;
        cout << "patchSize: "<<patchSize<<endl;
        cout << "miniMatch: "<< miniMatch<<endl;
        cout << "fastThreshold: "<< fastThreshold<<endl;
        cout << "miniNum: "<<miniNum<<endl;
        cout << "miniNumMono: "<< miniNumMono<<endl;
        cout << "farThreshold: "<< farThreshold<<endl;
        cout << "mk: "<<endl<< k <<endl;
        cout << "mDistCoef: "<<endl << mDistCoef<<endl;
        
        // new Rectifier pointer
        mpRectifierL = new Rectifier(configPath, mk, mDistCoef);
        mpRectifierR = new Rectifier(configPath, mk, mDistCoef);
        
        // new orb pointer
        mpOrbExtractorL = new OrbExtractor(num, scaleFactor, nLevels, iniThFAST, minThFAST);
        mpOrbExtractorR = new OrbExtractor(num, scaleFactor, nLevels, iniThFAST, minThFAST);
        
        mpMatcher = new Matcher(mb, mbf);
    }
    
    ImageProcessor::~ImageProcessor()
    {
        delete mpRectifierL;
        delete mpRectifierR;
        delete mpMatcher;
        
        mpRectifierL = NULL;
        mpRectifierR = NULL;
        mpMatcher = NULL;
    }
    
    // main function - featrue detect & match & triangulation
    // frameQualityFlag = false means info in this frame will not be sent to track thread
    featureInfo ImageProcessor::process(cv::Mat& imgL_, cv::Mat& imgR_)
    {
        TicToc timer;
        featureTrackTimes++;    // set to 1 for fisrt time, for print operation
        frameQualityFlag = false;
        
        // step 1. rect stereo imgs
        rectStereoImg(imgL_, imgR_);
        cv::Mat imgL = mpRectifierL->stereoRectL;
        cv::Mat imgR = mpRectifierR->stereoRectR;
        // mpRectifierL->test_visualizeRectStereo(imgL, imgR);
        
        // step 2. feature detection
        // get keypoints && desps
        // 0 means original version
        // 1 means selfe define version(more parallel threads)
        featureDetect(imgL, imgR, 1);
        
        // step 3. featrue matches between left and right cam
        // since the image is rectified, we can use epipolar line
        // matching method
        featureMatch(imgL, imgR, 1);
        
        // step 4. 
        if(mStereoKpL.size() > miniNum)
        {
            frameQualityFlag = true;
            
            mFeatureInfo.vecKps_camNorm = projectIntoCamNorm();
            mFeatureInfo.vecDepthes = mpMatcher->mDepth;
            mFeatureInfo.desp = mStereoDespL.clone();
            mFeatureInfo.img = imgL.clone();
        }
        else
        {
            frameQualityFlag = false;
        }
        
        // step 5. clear variables
        mStereoKpL.clear();
        mStereoKpR.clear();
        mStereoDespL.release();
        mStereoDespR.release();
        
        return mFeatureInfo;
    }
    
    void ImageProcessor::rectStereoImg(cv::Mat& imgL_, cv::Mat& imgR_)
    {
        thread threadRectLeft(&Rectifier::rectImage, mpRectifierL, imgL_, imgR_, 0);
        thread threadRectRight(&Rectifier::rectImage, mpRectifierR, imgL_, imgR_, 1);
        
        threadRectLeft.join();
        threadRectRight.join();
    }
    
    void ImageProcessor::featureDetect(cv::Mat& imgL_, cv::Mat& imgR_, int flag)
    {
        if (flag == 0)
        {
            thread threadLeft(&OrbExtractor::extractProcess,mpOrbExtractorL,imgL_);
            thread threadRight(&OrbExtractor::extractProcess,mpOrbExtractorR,imgR_);
            threadLeft.join();
            threadRight.join();
        }
        else if (flag == 1)
        {
            thread threadLeft(&OrbExtractor::extractProcess_thread,mpOrbExtractorL,imgL_);
            thread threadRight(&OrbExtractor::extractProcess_thread,mpOrbExtractorR,imgR_);
            threadLeft.join();
            threadRight.join();
        }
        
        mStereoKpL = mpOrbExtractorL->getOrbKeyPoints();
        mStereoKpR = mpOrbExtractorR->getOrbKeyPoints();
        mStereoDespL = mpOrbExtractorL->getOrbDesp();
        mStereoDespR = mpOrbExtractorR->getOrbDesp();

        // mpOrbExtractorL->test_DrawMatches(imgL_);
    }
    
    // 0 one trhead, 1 multi-thread
    void ImageProcessor::featureMatch(cv::Mat& imgL_, cv::Mat& imgR_,int flag)
    {
        if (flag == 0)
            mpMatcher->matchProcess(rows, mpOrbExtractorL, mpOrbExtractorR);
        else if (flag == 1)
            mpMatcher->matchProcess_thread(rows, mpOrbExtractorL, mpOrbExtractorR);
        
        // mpMatcher->test_visilization(mpOrbExtractorL, rows, cols, imgL_, imgR_);
    }
    
    vector<cv::Point2f> ImageProcessor::projectIntoCamNorm()
    {
        int N = mStereoKpL.size();
        
        vector<cv::Point2f> vecKps_CamNorm_Temp;
        vecKps_CamNorm_Temp.reserve(N);
        
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mStereoKpL[i];
            // project pixel points into camera norm coordinate
            float u = kp.pt.x;
            float v = kp.pt.y;
            float z = 1.0f;
            float invfx = 1/fx;
            float invfy = 1/fy;
            
            float x = (u - cx)*z*invfx;
            float y = (v - cy)*z*invfy;
            
            cv::Point2f point_CamNorm(x,y);
            cv::Point2f pixelPoint (u, v);
            vecKps_CamNorm_Temp.push_back(point_CamNorm);
        }
        
        return vecKps_CamNorm_Temp;
    }
    
}
