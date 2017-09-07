#include "imageProcess/ImageProcessor.h"
#include "track/Track.h"

#include "System.h"

using namespace std;

namespace StereoVO
{
    System::System(const string &configfliePath)
    {
        cout << "--- System init start" <<endl;
        
        imageProcessorStop = false;
        
        mpImageProcessor = new ImageProcessor(configfliePath);
        
        mpTrack = new Track();
        
        mpTrack->readParameters(configfliePath);
        
        mptTrack = new thread(&Track::process, mpTrack);
        
        cout << "--- system init finished"<<endl;
        // more threads can be added into this project whenever we want to expand it
    }
    
    
    System::~System()
    {
        mptTrack->join();
        
        if (mptTrack->joinable())
            mptTrack->join();
        
        delete mpImageProcessor;
        delete mpTrack;
        delete mptTrack;
        
        mpImageProcessor = NULL;
        mpTrack = NULL;
        mptTrack = NULL;
    }
    
    void System::processStereo(cv::Mat& imgL, cv::Mat& imgR)
    {
        featureInfo feature_info = mpImageProcessor->process(imgL, imgR);
        
        // send result to track thread
        setMsgtoTrack(feature_info);
    }
    
    void System::setMsgtoTrack(featureInfo& feature_info_)
    {
        if (mpImageProcessor->frameQualityFlag == true)
        {
            mpTrack->setFeatureBuff(feature_info_);
        }
    }
    
    void System::setStopFlag()
    {
        imageProcessorStop = true;
        
        mpTrack->setStopFlag(imageProcessorStop);
    }
    
}
