#include <unistd.h> 
#include <fstream>

#include "track/Track.h"
#include "System.h"
#include "imageProcess/Matcher.h"
#include "track/Frame.h"
#include "timer/tictoc.h"

using namespace std;

namespace StereoVO
{
    queue<featureInfo> Track::mFeatureBuff;
    
    Track::Track()
    {
        mTrackStatus = Init;
        stop_flag = false;
        buffEmpty_flag = true;
        mpMatcher = new Matcher();
    }
    
    Track::~Track()
    {
        delete mpMatcher;
        
        mpMatcher = NULL;
    }
    
    void Track::readParameters(const string& configPath)
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
        
        mbf = fsSettings["Camera.bf"];    // baseline time fx
        mb = mbf/fx;
        
        int threshold = fsSettings["ThDepth"];
        
        farThreshold = mb *threshold*1.2; 
        
        cv::Mat k = cv::Mat::eye(3, 3, CV_32F);
        k.at<float>(0,0) = fx;
        k.at<float>(0,2) = cx;
        k.at<float>(1,1) = fy;
        k.at<float>(1,2) = cy;
        k.copyTo(mk);
    }
    
    void Track::process()
    {
        int id = 0;
        
        while(1)
        {
            // step 1. wait untill get msgBuff
            unique_lock<mutex> lk_readBuff(mMutexFeatureBuff);
            
            con_featureBuff.wait(lk_readBuff, checkFeatureBuffnotEmpty);
            
            // step 2. creat curr frame
            creatCurrFrame();
            lk_readBuff.unlock();

            // step 3. match current && last msg keypoints
            if( id != 0)
            {
                mpMatcher->matcheTwoFrames(mCurrFrame, mLastFrame, fx, fy, cx, cy);
                
                // for test purpose - check the result of MatchTwoFrames
                cv::Mat left = mLastFrame.getImg();
                cv::Mat right = mCurrFrame.getImg();
                test_visulFrameMatchResult(left, right);
                
                computePose();
            }
            
            mLastFrame = Frame(mCurrFrame);
            id++;
            
            {
                unique_lock<mutex> lk_stop(mMutexStop);
                if (stop_flag == true)
                {
                    cout << "--- System will be shut down, close track thread"<<endl;
                    break;
                }
            }
        }
    }
    
    void Track::creatCurrFrame()
    {
        featureInfo temp = mFeatureBuff.front();
        mFeatureBuff.pop();
        
        mCurrFrame = Frame(temp, farThreshold);
        
        if (mTrackStatus == Init)
        {
            mTrackStatus = Norm;
            mCurrFrame.setPose();
        }
        
        mCurrFrame.getCameraParameter(fx, fy, cx, cy, mbf, mb, farThreshold, mk);
    }
    
    void Track::computePose()
    {
        vector<pair<cv::Point2f, cv::Point2f>> vecMathes_camNorm = mpMatcher->getMatch2D2D_norm();   // last & current mathes in camera norm coordinate
        vector<pair<cv::Point2f, cv::Point2f>> vecMathes_pixel = mpMatcher->getMatch2D2D_pixel(); 
        vector<int> vecMathedID = mpMatcher->getMatchedID();    // store kps ID for lastframe
        vector<float> vecDepthes = mLastFrame.getDepthes();     // store depth for lastframe, -1 means no depth
        
        vector<cv::Point3f> vecPoints_world;
        vector<cv::Point2f> vecPoints_pixel;
        for (int i = 0; i < vecMathes_camNorm.size(); i++)
        {
            cv::Point2f point_norm, point_pixel;
            cv::Point3f point_world;
            int corrs_id;
            float z;
            
            point_norm = vecMathes_camNorm[i].first;        // last frame
            corrs_id = vecMathedID[i];
            z = vecDepthes[corrs_id];
            
            point_world = pointsCamnorm2World(point_norm, z, i);
            
            point_pixel = vecMathes_pixel[i].second;        // curr frame
            
            vecPoints_world.push_back(point_world);
            vecPoints_pixel.push_back(point_pixel);
        }
        
        cv::Mat R,t;
        
        solvePnpProblem(vecPoints_world, vecPoints_pixel, R, t);
        
        // uncommon below code if you do not choose iteration for solvePNP
        // poseOptimizor.poseOptimze(vecPoints_world, vecPoints_pixel, mk, R, t);
        
        if (R.type() != CV_32F || t.type() != CV_32F)
        {
            R.convertTo(R, CV_32F);
            t.convertTo(t, CV_32F);
        }

        mCurrFrame.setRotation(R);
        mCurrFrame.setTranslation(t);
    }
    
    // for last frame
    cv::Point3f Track::pointsCamnorm2World(cv::Point2f& point_norm_, float z_, int i)
    {
        cv::Point3f point_world_;

        float x = point_norm_.x*z_;
        float y = point_norm_.y*z_;
        float z = z_;
        
        cv::Mat point_cam_mat = (cv::Mat_<float>(3,1) << x, y, z);
        
        cv::Mat Rwc = mLastFrame.getRwc();
        cv::Mat twc = mLastFrame.gettwc();
        
        cv::Mat point_world_mat = Rwc*point_cam_mat+twc;
        
        point_world_.x = point_world_mat.at<float>(0,0);
        point_world_.y = point_world_mat.at<float>(1,0);
        point_world_.z = point_world_mat.at<float>(2,0);
        
        return point_world_;
    }
    
    void Track::solvePnpProblem(vector<cv::Point3f>& points3D, vector<cv::Point2f>& points2D, cv::Mat& R, cv::Mat& t)
    {
        cv::Mat r;

        bool success = cv::solvePnP(points3D, points2D, mk, cv::Mat(), r, t, false, cv::SOLVEPNP_EPNP);
        
        if (success == false)
        {
            cerr<< "ERROR: solvePNP OpenCV operation wrong"<< endl;
            exit(-1);
        }
        cv::Rodrigues (r, R); 
    }
   
    void Track::setFeatureBuff(featureInfo& info)
    {
        unique_lock<mutex> lk_setBuff(mMutexFeatureBuff);
        
        mFeatureBuff.push(info);
        
        lk_setBuff.unlock();
        
        con_featureBuff.notify_one();
    }
    
    void Track::test_visulFrameMatchResult(cv::Mat& left, cv::Mat& right)
    {
        vector<pair<cv::Point2f, cv::Point2f>> match_2D_2D = mpMatcher->getMatch2D2D_norm();

        // test visulization
        cv::Mat imgL = left.clone();
        cv::Mat imgR = right.clone();
        
        int imgsizeH = imgL.rows;
        int imgsizeW = imgL.cols;
        cv::Mat showImage(imgsizeH,2*imgsizeW,imgL.type());
        cv::Rect Left(0,0,imgsizeW,imgsizeH);
        cv::Rect Right(imgsizeW,0,imgsizeW,imgsizeH);
        
        imgL.copyTo(showImage(Left));
        imgR.copyTo(showImage(Right));

        for (int num = 0; num < match_2D_2D.size(); num++)
        {
            int r = 5;

            float uL = match_2D_2D[num].first.x*fx +cx;
            float vL = match_2D_2D[num].first.y*fy +cy;
            
            float uC = match_2D_2D[num].second.x*fx +cx + imgsizeW;
            float vC = match_2D_2D[num].second.y*fy +cy;
            
            cv::Point centerL = cv::Point(uL,vL);
            cv::Point centerR = cv::Point(uC,vC);
            
            cv::circle(showImage, centerL, r, cv::Scalar(227,168,105));
            cv::circle(showImage, centerR, r, cv::Scalar(227,168,105));
            cv::line(showImage, centerL, centerR, cv::Scalar(227,168,105));
        }

        cv::imshow("Match_Last_Current", showImage);
        cv::waitKey(1); 
    }
    
    void Track::setStopFlag(bool flag)
    {
        mMutexStop.lock();
        
        stop_flag = flag;
        
        mMutexStop.unlock();
    }
    

    bool Track::checkFeatureBuffnotEmpty()
    {
        if (mFeatureBuff.empty())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}

