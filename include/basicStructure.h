#pragma once

#ifndef BASICSTRUCTURE_H
#define BASICSTRUCTURE_H

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

using namespace std;

namespace StereoVO
{
    struct keypoint_SelfDefine
    {
        int id;             // id of kps in this frame
        cv::KeyPoint kps;   // keypoint info
        bool hasMapPoint;   // matched(true) or not matched(false)
        float corrsDepth;   // depth for corrs 3d points
    };
    
    struct imgInfo
    {
        vector<keypoint_SelfDefine> vecKps_info;
        vector<pair<int, Eigen::Vector3f>> vecpoints3D;
        cv::Mat desp;
        cv::Mat img;        // for test, can be deleted if u want
    };
    
    struct featurePoint2D
    {
        int id_point;        // point id in current frame
        bool hasMapPoint; 
        cv::KeyPoint kps;    // keypoint info       
        float corrsDepth;    // depth for corrs 3d points
    };
    
    struct featureInfo
    {
        vector<cv::Point2f> vecKps_camNorm;
        vector<float> vecDepthes;
        cv::Mat desp;
        cv::Mat img;
    };
}
#endif
