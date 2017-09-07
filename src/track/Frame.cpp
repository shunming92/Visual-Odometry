#include <iterator>
#include <unistd.h> 
#include <fstream>

#include "track/Frame.h"

using namespace std;

namespace StereoVO
{
    Frame::Frame()
    {
        print = true; //debug
    }
    
    Frame::Frame(featureInfo& featureMsg, int farThreshold_):farThreshold(farThreshold_)
    {
        vecKps_camNorm = featureMsg.vecKps_camNorm;
        mdesp = featureMsg.desp.clone();
        mimg = featureMsg.img.clone();
        
        int N = vecKps_camNorm.size();
        
        vector<float> depth;
        depth.reserve(N);
        
        for (int i = 0; i < N; i++)
        {
            float depth_value = featureMsg.vecDepthes[i];
            if (depth_value > farThreshold)
                depth_value = -1;
            
            depth.push_back(depth_value);
        }
        
        vecDepthes = depth;
    }
        
    Frame::Frame(const Frame &frame):
    vecKps_camNorm(frame.vecKps_camNorm), vecDepthes(frame.vecDepthes), mdesp(frame.mdesp.clone()), 
    mimg(frame.mimg.clone()),mTcw(frame.mTcw.clone()), mTwc(frame.mTwc.clone()), mRcw(frame.mRcw.clone()),
    mRwc(frame.mRwc.clone()), mtcw(frame.mtcw.clone()), mtwc(frame.mtwc.clone())
    {
    }
    
    Frame::~Frame()
    {
    }
    
    void Frame::getCameraParameter(float fx_, float fy_, float cx_, float cy_, float mbf_, float mb_, int farThreshold_, cv::Mat& k_)
    {
        fx = fx_; fy = fy_; 
        cx = cx_; cy = cy_;
        
        mb = mb_; mbf = mbf_;
        farThreshold = farThreshold_;
        
        k = k_.clone();
        
        invfx = 1/ fx; invfy = 1/ fy;
    }

    void Frame::setPose()
    {
        mRcw = cv::Mat::eye(3,3,CV_32F);
        mRwc = mRcw.t();
        mtcw = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, 0.0f);
        mtwc = -mRcw.t()*mtcw;
    }
    
    cv::Mat Frame::getImg()
    {
        return mimg;
    }

    cv::Mat Frame::getDesp()
    {
        return mdesp;
    }
    
    vector<cv::Point2f> Frame::getKps_camNorm()
    {
        return vecKps_camNorm;
    }
    
    vector<float> Frame::getDepthes()
    {
        return vecDepthes;
    }
    
    void Frame::setRotation(cv::Mat& R)
    {
        mRcw = R.clone();
        mRwc = mRcw.t();
    }
    
    void Frame::setTranslation(cv::Mat& t)
    {
        mtcw = t.clone();
        mtwc = -mRcw.t()*mtcw;
    }
    
    cv::Mat Frame::getRcw()
    {
        return mRcw;
    }
    
    cv::Mat Frame::getRwc()
    {
        return mRwc;
    }
    
    cv::Mat Frame::gettwc()
    {
        return mtwc;
    }
    
    cv::Mat Frame::gettcw()
    {
        return mtcw;
    }
    
}
  
  
//       cv::Point3f Frame::camera2world(cv::Point3f& point3D, bool print)
//     {
//         const float xc = point3D.x;
//         const float yc = point3D.y;
//         const float zc = point3D.z;
//         
//         Eigen::Vector3f point_c;
//         point_c << xc, yc, zc;
//         
//         Eigen::Matrix3f Rwc;
//         Eigen::Vector3f twc;
//         
//         Rwc << mRwc.at<double> ( 0,0 ), mRwc.at<double> ( 0,1 ), mRwc.at<double> ( 0,2 ),
//                mRwc.at<double> ( 1,0 ), mRwc.at<double> ( 1,1 ), mRwc.at<double> ( 1,2 ),
//                mRwc.at<double> ( 2,0 ), mRwc.at<double> ( 2,1 ), mRwc.at<double> ( 2,2 );
//         
//         twc << mtwc.at<double> ( 0,0 ), mtwc.at<double> ( 1,0 ), mtwc.at<double> ( 2,0 );
//         
//         Eigen::Vector3f point_w_Eigen = Rwc * point_c + twc;
//         float xw = point_w_Eigen(0);
//         float yw = point_w_Eigen(1);
//         float zw = point_w_Eigen(2);
//         
//         cv::Point3f point_w(xw, yw,zw);
//         return point_w;
//     }
//     
//     // change into camera norm coordinate
//     cv::Point3f Frame::pixel2camera(cv::Point2f& point2D)
//     {
//         float u = point2D.x;
//         float v = point2D.y;
//         float z = 1.0f;
//         
//         float x = (u - cx)*z*invfx;
//         float y = (v - cy)*z*invfy;
//         
//         cv::Point3f point_c_norm(x, y, z);
//         
//         return point_c_norm;
//     }
  
  
//     void Frame::addMapPoint(int i, imgInfo& imgmsg_)
//     {
//         MapPoint mapPointTemp;
//         mapPointTemp.id_corresFeature = imgmsg_.vecpoints3D[i].first;
//         mapPointTemp.mapPoint = imgmsg_.vecpoints3D[i].second;
//         mapPointTemp.desp = mdesp.row(i);
//         mMapPoints.insert(pair<int,MapPoint>(i, mapPointTemp));
//     }
//     
//     void Frame::resetFarPoints(int i, imgInfo& imgmsg_)
//     {
//         // keypoint id corres to this 3d points
//         int id = imgmsg_.vecpoints3D[i].first;
//         mfeatPoints[id].hasMapPoint = false;
//         mfeatPoints[id].corrsDepth = -1;
//     }
//     
//     void Frame::setPose(cv::Mat Tcw)
//     {
//         mTcw = Tcw.clone();
//         updatePoseMatrices();
//     }
//     
//     void Frame::updatePoseMatrices()
//     { 
//         mRcw = mTcw.rowRange(0,3).colRange(0,3);
//         mRwc = mRcw.t();
//         mtcw = mTcw.rowRange(0,3).col(3);
//         mtwc = -mRcw.t()*mtcw;
//     }


//-----------------------------------------------------------------------------------------------------------------------
//     Frame::Frame(imgInfo& imgmsg_, int farThreshold_, int frameCount_):farThreshold(farThreshold_),id(frameCount_)
//     {
//         int size = imgmsg_.desp.rows;
//         mfeatPoints.resize(size);
// 
//         // store feature points info
//         for (int i = 0; i < size; i++)
//         {
//             mfeatPoints[i].id_point = imgmsg_.vecKps_info[i].id;
//             mfeatPoints[i].hasMapPoint = imgmsg_.vecKps_info[i].hasMapPoint;
//             mfeatPoints[i].kps = imgmsg_.vecKps_info[i].kps;
//             mfeatPoints[i].corrsDepth = imgmsg_.vecKps_info[i].corrsDepth;
//         }
//         
//         // store corrs desp info
//         mdesp = imgmsg_.desp.clone();
//         
//         // store map points info
//         int counter = 0;
//         for (int i = 0; i < imgmsg_.vecpoints3D.size(); i++)
//         {
//             if(imgmsg_.vecpoints3D[i].second(2) <= farThreshold)
//             {
//                 // add into mapPoint in current frame
//                 // if z is less than our Threshold
//                 counter++;
//                 addMapPoint(counter, imgmsg_);
//             }
//             else
//             {
//                 // otherwise, reset corrs mfeatPoints
//                 resetFarPoints(i,imgmsg_);
//             }
//         }
//         
//     }
/*


void Frame::addMapPoint(int i, imgInfo& imgmsg_)
    {
        MapPoint mapPointTemp;
        mapPointTemp.id_corresFeature = imgmsg_.vecpoints3D[i].first;
        mapPointTemp.mapPoint = imgmsg_.vecpoints3D[i].second;
        mapPointTemp.desp = mdesp.row(i);
        mMapPoints.insert(pair<int,MapPoint>(i, mapPointTemp));
    }
    
    void Frame::resetFarPoints(int i, imgInfo& imgmsg_)
    {
        // keypoint id corres to this 3d points
        int id = imgmsg_.vecpoints3D[i].first;
        mfeatPoints[id].hasMapPoint = false;
        mfeatPoints[id].corrsDepth = -1;
    }
    
    void Frame::setPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        updatePoseMatrices();
    }
    
    void Frame::updatePoseMatrices()
    { 
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mtwc = -mRcw.t()*mtcw;
    }
// */  
