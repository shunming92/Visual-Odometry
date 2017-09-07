/*
 * This file is mainly based on ORB-SLAM2 programed by  Raul Mur-Artal, 
 * Juan D. Tardos, J. M. M. Montiel and Dorian Galvez-Lopez (DBoW2).
 * The project link: https://github.com/raulmur/ORB_SLAM2
 * 
 * What I have done in this part is using parallel thread methods to 
 * speed up the stereo matching (details in function extractProcess_thread).
 * 
 * Also I have changed the structure of original code to adjust to my
 * project.
 * 
 * by Shunming Li
 * 
 */
#include <unistd.h> 
#include <fstream>

#include "imageProcess/OrbExtractor.h"
#include "imageProcess/Matcher.h"
#include "timer/tictoc.h"

using namespace std;

namespace StereoVO
{
    Matcher::Matcher()
    {
    }
    
    Matcher::Matcher(int mb_, int mbf_):mb(mb_),mbf(mbf_)
    {
    }
    
    Matcher::~Matcher()
    {
    }
    
    // in stereo match, the images are calibrated. Thus, we only consider u value
    void Matcher::matchProcess(int rows, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR)
    {
        vector<cv::KeyPoint>kpsL = mpOrbExtractorL->getOrbKeyPoints();
        vector<cv::KeyPoint>kpsR = mpOrbExtractorR->getOrbKeyPoints();
        if(kpsL.size() != 0 && kpsR.size() != 0)
        {
            cv::Mat despL, despR;
            
            despL = mpOrbExtractorL->getOrbDesp().clone();
            despR = mpOrbExtractorR->getOrbDesp().clone();
            
            int N = kpsL.size();
            
            muR = vector<float>(N, -1.0f);
            mDepth = vector<float>(N, -1.0f);
            
            const int thOrbDist = (TH_HIGH+TH_LOW)/2;
            
            const int nRows = rows;
            
            //Assign keypoints to row table
            vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
            
            for(int i=0; i<nRows; i++)
                vRowIndices[i].reserve(200);
            
            const int Nr = kpsR.size();
            
            for(int iR=0; iR<Nr; iR++)
            {
                const cv::KeyPoint &kp = kpsR[iR];
                const float &kpY = kp.pt.y;
                const float r = 2.0f*mpOrbExtractorL->mvScaleFactor[kpsR[iR].octave];
                const int maxr = ceil(kpY+r);
                const int minr = floor(kpY-r);
                
                for(int yi=minr;yi<=maxr;yi++)
                    vRowIndices[yi].push_back(iR);
            }
            
            // Set limits for search
            const float minZ = mb;
            const float minD = 0.0f;
            const float maxD = mbf/minZ;
            
            // For each left keypoint search a match in the right image
            vector<pair<int, int> > vDistIdx;
            vDistIdx.reserve(N);
            
            for(int iL=0; iL<N; iL++)
            {
                const cv::KeyPoint &kpL = kpsL[iL];
                const int &levelL = kpL.octave;
                const float &vL = kpL.pt.y;
                const float &uL = kpL.pt.x;
                
                const vector<size_t> &vCandidates = vRowIndices[vL];
                
                if(vCandidates.empty())
                    continue;
                
                const float minU = uL-maxD;
                const float maxU = uL-minD;
                
                if(maxU<0)
                    continue;
                
                int bestDist = TH_HIGH;
                size_t bestIdxR = 0;
                
                const cv::Mat &dL =despL.row(iL);
                
                // Compare descriptor to right keypoints
                for(size_t iC=0; iC<vCandidates.size(); iC++)
                {
                    const size_t iR = vCandidates[iC];
                    const cv::KeyPoint &kpR = kpsR[iR];
                    
                    if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                        continue;
                    
                    const float &uR = kpR.pt.x;
                    
                    if(uR>=minU && uR<=maxU)
                    {
                        const cv::Mat &dR = despR.row(iR);
                        const int dist = descriptorDistance(dL,dR);
                        
                        if(dist<bestDist)
                        {
                            bestDist = dist;
                            bestIdxR = iR;
                        }
                    }
                }
                
                // Subpixel match by correlation
                if(bestDist<thOrbDist)
                {
                    // coordinates in image pyramid at keypoint scale
                    const float uR0 = kpsR[bestIdxR].pt.x;
                    const float scaleFactor = mpOrbExtractorL->mvInvScaleFactor[kpL.octave];
                    const float scaleduL = round(kpL.pt.x*scaleFactor);
                    const float scaledvL = round(kpL.pt.y*scaleFactor);
                    const float scaleduR0 = round(uR0*scaleFactor);
                    
                    // sliding window search
                    const int w = 5;
                    cv::Mat IL = mpOrbExtractorL->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
                    IL.convertTo(IL,CV_32F);
                    IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
                    
                    int bestDist = INT_MAX;
                    int bestincR = 0;
                    const int L = 5;
                    vector<float> vDists;
                    vDists.resize(2*L+1);
                    
                    const float iniu = scaleduR0+L-w;
                    const float endu = scaleduR0+L+w+1;
                    if(iniu<0 || endu >= mpOrbExtractorR->mvImagePyramid[kpL.octave].cols)
                        continue;
                    
                    for(int incR=-L; incR<=+L; incR++)
                    {
                        cv::Mat IR = mpOrbExtractorR->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                        IR.convertTo(IR,CV_32F);
                        IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                        
                        float dist = cv::norm(IL,IR,cv::NORM_L1);
                        if(dist<bestDist)
                        {
                            bestDist =  dist;
                            bestincR = incR;
                        }
                        
                        vDists[L+incR] = dist;
                    }
                    
                    if(bestincR==-L || bestincR==L)
                        continue;
                    
                    // Sub-pixel match (Parabola fitting)
                    const float dist1 = vDists[L+bestincR-1];
                    const float dist2 = vDists[L+bestincR];
                    const float dist3 = vDists[L+bestincR+1];
                    
                    const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
                    
                    if(deltaR<-1 || deltaR>1)
                        continue;
                    
                    // Re-scaled coordinate
                    float bestuR = mpOrbExtractorL->mvScaleFactor[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
                    
                    float disparity = (uL-bestuR);
                    
                    if(disparity>=minD && disparity<maxD)
                    {
                        if(disparity<=0)
                        {
                            disparity=0.01;
                            bestuR = uL-0.01;
                        }
                        mDepth[iL]=mbf/disparity;
                        // cout << "--- mDepth "<< o<< ": "<<mDepth[o]<<endl;
                        muR[iL] = bestuR;
                        vDistIdx.push_back(pair<int,int>(bestDist,iL));
                    }
                }
            }
            
            sort(vDistIdx.begin(),vDistIdx.end());
            const float median = vDistIdx[vDistIdx.size()/2].first;
            const float thDist = 1.5f*1.4f*median;
            
            for(int i=vDistIdx.size()-1;i>=0;i--)
            {
                if(vDistIdx[i].first<thDist)
                {
                    break;
                }
                else
                {
                    muR[vDistIdx[i].second]=-1;
                    mDepth[vDistIdx[i].second]=-1;
                }
            }
        }
    }
    
    void Matcher::matchProcess_thread(int rows, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR)
    {
        vector<cv::KeyPoint>kpsL = mpOrbExtractorL->getOrbKeyPoints();
        vector<cv::KeyPoint>kpsR = mpOrbExtractorR->getOrbKeyPoints();
        if(kpsL.size() != 0 && kpsR.size() != 0)
        {
            cv::Mat despL, despR;
            
            despL = mpOrbExtractorL->getOrbDesp().clone();
            despR = mpOrbExtractorR->getOrbDesp().clone();
            
            int N = kpsL.size();
            
            muR = vector<float>(N, -1.0f);
            mDepth = vector<float>(N, -1.0f);
            
            const int thOrbDist = (TH_HIGH+TH_LOW)/2;
            
            const int nRows = rows;
            
            //Assign keypoints to row table
            vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
            
            for(int i=0; i<nRows; i++)
                vRowIndices[i].reserve(200);
            
            const int Nr = kpsR.size();
            
            for(int iR=0; iR<Nr; iR++)
            {
                const cv::KeyPoint &kp = kpsR[iR];
                const float &kpY = kp.pt.y;
                const float r = 2.0f*mpOrbExtractorL->mvScaleFactor[kpsR[iR].octave];
                const int maxr = ceil(kpY+r);
                const int minr = floor(kpY-r);
                
                for(int yi=minr;yi<=maxr;yi++)
                    vRowIndices[yi].push_back(iR);
            }
            
            // Set limits for search
            const float minZ = mb;
            const float minD = 0.0f;
            const float maxD = mbf/minZ;
            
            // For each left keypoint search a match in the right image
            vector<pair<int, int> > vDistIdx;
            vDistIdx.reserve(N);
            
            int threadnum = 4;
            thread stereoMatches1(&Matcher::stereoMatches, this, threadnum, 0, N, ref(vRowIndices), maxD, minD, thOrbDist, ref(vDistIdx), ref(kpsL), ref(kpsR), ref(despL), ref(despR), mpOrbExtractorL, mpOrbExtractorR);
            thread stereoMatches2(&Matcher::stereoMatches, this, threadnum, 1, N, ref(vRowIndices), maxD, minD, thOrbDist, ref(vDistIdx), ref(kpsL), ref(kpsR), ref(despL), ref(despR), mpOrbExtractorL, mpOrbExtractorR);
            thread stereoMatches3(&Matcher::stereoMatches, this, threadnum, 2, N, ref(vRowIndices), maxD, minD, thOrbDist, ref(vDistIdx), ref(kpsL), ref(kpsR), ref(despL), ref(despR), mpOrbExtractorL, mpOrbExtractorR);
            thread stereoMatches4(&Matcher::stereoMatches, this, threadnum, 3, N, ref(vRowIndices), maxD, minD, thOrbDist, ref(vDistIdx), ref(kpsL), ref(kpsR), ref(despL), ref(despR), mpOrbExtractorL, mpOrbExtractorR);

            stereoMatches1.join();
            stereoMatches2.join();
            stereoMatches3.join();
            stereoMatches4.join();
            
            sort(vDistIdx.begin(),vDistIdx.end());
            const float median = vDistIdx[vDistIdx.size()/2].first;
            const float thDist = 1.5f*1.4f*median;
            
            for(int i=vDistIdx.size()-1;i>=0;i--)
            {
                if(vDistIdx[i].first<thDist)
                {
                    break;
                }
                else
                {
                    muR[vDistIdx[i].second]=-1;
                    mDepth[vDistIdx[i].second]=-1;
                }
            }
        }
        
    }
    
    void Matcher::stereoMatches(int threadnumber,int ID,int N, vector<vector<size_t>> &vRowIndices, float maxD, float minD, int thOrbDist, vector<pair<int, int> > &vDistIdx, vector<cv::KeyPoint>& kpsL, vector<cv::KeyPoint>& kpsR, cv::Mat& despL, cv::Mat despR, OrbExtractor* mpOrbExtractorL, OrbExtractor* mpOrbExtractorR)
    {
        for(int iL = N/threadnumber*ID; iL < N/threadnumber*(ID+1); iL++)
        {
            const cv::KeyPoint &kpL = kpsL[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;
            
            const vector<size_t> &vCandidates = vRowIndices[vL];
            
            if(vCandidates.empty())
                continue;
            
            const float minU = uL-maxD;
            const float maxU = uL-minD;
            
            if(maxU<0)
                continue;
            
            int bestDist = TH_HIGH;
            size_t bestIdxR = 0;
            
            const cv::Mat &dL =despL.row(iL);
            
            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = kpsR[iR];
                
                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;
                
                const float &uR = kpR.pt.x;
                
                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = despR.row(iR);
                    const int dist = descriptorDistance(dL,dR);
                    
                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }
            
            // Subpixel match by correlation
            if(bestDist<thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = kpsR[bestIdxR].pt.x;
                const float scaleFactor = mpOrbExtractorL->mvInvScaleFactor[kpL.octave];
                const float scaleduL = round(kpL.pt.x*scaleFactor);
                const float scaledvL = round(kpL.pt.y*scaleFactor);
                const float scaleduR0 = round(uR0*scaleFactor);
                
                // sliding window search
                const int w = 5;
                cv::Mat IL = mpOrbExtractorL->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
                IL.convertTo(IL,CV_32F);
                IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
                
                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2*L+1);
                
                const float iniu = scaleduR0+L-w;
                const float endu = scaleduR0+L+w+1;
                if(iniu<0 || endu >= mpOrbExtractorR->mvImagePyramid[kpL.octave].cols)
                    continue;
                
                for(int incR=-L; incR<=+L; incR++)
                {
                    cv::Mat IR = mpOrbExtractorR->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                    IR.convertTo(IR,CV_32F);
                    IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                    
                    float dist = cv::norm(IL,IR,cv::NORM_L1);
                    if(dist<bestDist)
                    {
                        bestDist =  dist;
                        bestincR = incR;
                    }
                    
                    vDists[L+incR] = dist;
                }
                
                if(bestincR==-L || bestincR==L)
                    continue;
                
                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L+bestincR-1];
                const float dist2 = vDists[L+bestincR];
                const float dist3 = vDists[L+bestincR+1];
                
                const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
                
                if(deltaR<-1 || deltaR>1)
                    continue;
                
                // Re-scaled coordinate
                float bestuR = mpOrbExtractorL->mvScaleFactor[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
                
                float disparity = (uL-bestuR);
                
                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        bestuR = uL-0.01;
                    }
                    mDepth[iL]=mbf/disparity;
                    // cout << "--- mDepth "<< o<< ": "<<mDepth[o]<<endl;
                    muR[iL] = bestuR;
                    vDistIdx.push_back(pair<int,int>(bestDist,iL));
                }
            }
        }
    }
    
    int Matcher::descriptorDistance(const cv::Mat& a, const cv::Mat& b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();
        
        int dist=0;
        
        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }
        
        return dist;
    }
    
    void Matcher::computeImageBounds(cv::Mat img_)
    {
        mnMinX = 0.0f;
        mnMaxX = img_.cols;
        mnMinY = 0.0f;
        mnMaxY = img_.rows;
    }
    
    void Matcher::assignFeaturesToGrid(OrbExtractor* mpOrbExtractorL)
    {
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
        
        vector<cv::KeyPoint> kpsL = mpOrbExtractorL->getOrbKeyPoints();
        
        int featureNum = kpsL.size();
        int nReserve = featureNum/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        if (nReserve == 0)
            nReserve = 1;
        
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);
            
            for(int i=0;i<featureNum;i++)
            {
                const cv::KeyPoint &kp = kpsL[i];
                
                int nGridPosX, nGridPosY;
                if(posInGrid(kp,nGridPosX,nGridPosY))
                    mGrid[nGridPosX][nGridPosY].push_back(i);
            }
    }
    
    bool Matcher::posInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
        
        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        
        return true;
    }
    
    void Matcher::test_visilization(OrbExtractor* mpOrbExtractorL, int rows, int cols, cv::Mat& imgL_, cv::Mat& imgR_)
    {
        cv::Mat imgL = imgL_.clone();
        cv::Mat imgR = imgR_.clone();
        vector<cv::KeyPoint> kpsL = mpOrbExtractorL->getOrbKeyPoints();
        
        int imgsizeH = rows;
        int imgsizeW = cols;
        cv::Mat showImage(imgsizeH,2*imgsizeW,imgL.type());
        cv::Rect Left(0,0,imgsizeW,imgsizeH);
        cv::Rect Right(imgsizeW,0,imgsizeW,imgsizeH);
        
        imgL.copyTo(showImage(Left));
        imgR.copyTo(showImage(Right));
        
        for (int i = 0; i < kpsL.size(); i++)
        {
            if(muR[i] != -1)
            {
                int r = 5;
                cv::Point centerL = cv::Point(kpsL[i].pt.x, kpsL[i].pt.y);
                cv::Point centerR = cv::Point(muR[i]+cols, kpsL[i].pt.y);
                cv::circle(showImage, centerL, r, cv::Scalar(227,168,105));
                cv::circle(showImage, centerR, r, cv::Scalar(227,168,105));
                cv::line(showImage, centerL, centerR, cv::Scalar(227,168,105));
            }
        }
        cv::imshow("Match_SelfDefine", showImage);
        cv::waitKey(30);        
    }
    
    // only match 3d points in lastFrame with 2d points in currFrame
    int Matcher::matcheTwoFrames(Frame& currFrame, Frame& lastFrame, float fx, float fy, float cx, float cy)
    {
        int matchedNum;
        
        vector<pair<cv::Point2f, cv::Point2f>> vecMatched_original;
        vector<pair<cv::Point2f, cv::Point2f>> vecMatched_outRemove;
        
        vector<int> vecMatchedID_last;              // used to find corr 3d points
        vector<int> vecMatchedID_last_outRemove;    // used to find corr 3d points
        
        findBestMatchInCurrFrame(currFrame, lastFrame, vecMatched_original,vecMatchedID_last);
        
        matchedNum = vecMatched_original.size();

        removeOutlinerRansac(vecMatched_original, vecMatchedID_last, fx, fy, cx, cy);

        // cout << "----------------mvecMatch_2D_2D: "<<mvecMatch_2D_2D.size()<<endl; 
        // cout << "----------------mvecMatch_3D_2D: "<<mvecMatch_3D_2D.size()<<endl;
        return matchedNum;
    }
    
    void Matcher::findBestMatchInCurrFrame(Frame& currFrame, Frame& lastFrame, vector<pair<cv::Point2f, cv::Point2f>>& matchedPoints, vector<int>& matchedID_last)
    {
        vector<cv::Point2f> kps_last = lastFrame.getKps_camNorm();
        vector<cv::Point2f> kps_curr = currFrame.getKps_camNorm();
        
        vector<float> depthes_last = lastFrame.getDepthes();
        
        cv::Mat desp_last = lastFrame.getDesp();
        cv::Mat desp_curr = currFrame.getDesp();
        
        int N = kps_last.size();
        for(int i = 0; i < N; i++)
        {
            if (depthes_last[i] != -1)
            {
                int bestDist = TH_HIGH;
                int bestIdxCurr = 0;
            
                cv::Mat despl = desp_last.row(i);
                
                for (int j = 0; j < kps_curr.size(); j++)
                {
                    cv::Mat despc = desp_curr.row(j);

                    // calculate distance between these two kps
                    const int dist = descriptorDistance(despl, despc);
                    
                    if(dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxCurr = j;
                    }
                }
                cv::Point2f kpl, kpc, pixelL, pixelC;
                kpl = kps_last[i];
                kpc = kps_curr[bestIdxCurr];
                
                matchedPoints.push_back(pair<cv::Point2f, cv::Point2f>(kpl, kpc));
                matchedID_last.push_back(i);    // kps id for lastFrame
            }
        }
    }
    
    
    void Matcher::removeOutlinerRansac( vector<pair<cv::Point2f, cv::Point2f>>& vecMatched_original_, vector<int>& vecMatchedID_last_, float fx, float fy, float cx, float cy)
    {
        int N = vecMatched_original_.size();
        
        vector<pair<cv::Point2f, cv::Point2f>> matches_remove;
        vector<pair<cv::Point2f, cv::Point2f>> matchespixel_remove;
        vector<int> mathes_id_remove;
        vector<cv::Point2f> pointsL, pointsC;
        
        matches_remove.reserve(N);
        matchespixel_remove.reserve(N);
        mathes_id_remove.reserve(N);
        pointsL.reserve(N);
        pointsC.reserve(N);
        
        for (int i = 0; i < N; i ++)
        {
            // project from cam norm into cam pixel
            float xL = vecMatched_original_[i].first.x;
            float yL = vecMatched_original_[i].first.y;
            
            float xC = vecMatched_original_[i].second.x;
            float yC = vecMatched_original_[i].second.y;
            
            float uL = fx*xL + cx;
            float vL = fy*yL + cy;
            float uC = fx*xC + cx;
            float vC = fy*yC + cy;
            
            cv::Point2f pointL(uL, vL);
            cv::Point2f pointC(uC, vC);
            
            pointsL.push_back(pointL);
            pointsC.push_back(pointC);
        }
        
        vector<uchar> status;
        cv::findFundamentalMat(pointsL, pointsC, cv::FM_RANSAC, 1.0, 0.99, status);
        
        int num = 0;
        for (int i = 0; i < vecMatched_original_.size(); i++)
        {
            if (status[i] != 0)
            {
                if (abs(pointsL[i].y - pointsC[i].y) < 100)
                {
                    matches_remove.push_back(vecMatched_original_[i]);  // cam norm coordinate
                    mathes_id_remove.push_back(vecMatchedID_last_[i]);  // kps id for lastFrame, for depth obtain
                    matchespixel_remove.push_back(pair<cv::Point2f, cv::Point2f>(pointsL[i], pointsC[i]));
                    // cout << "--- remove id: "<< vecMatchedID_last_[i]<<endl;
                    // cout << "--- matche point for "<< "corrs_id "<< vecMatchedID_last_[i]<< " is: "<< vecMatched_original_[i].first<<endl;
                    num++;
                }
            }
        }

        mvecMatch_2D_2D_norm = matches_remove;
        mvecMatch_2D_2D_pixel = matchespixel_remove;
        mvecMatchedID = mathes_id_remove;
    }
    
    vector<pair<cv::Point2f, cv::Point2f>> Matcher::getMatch2D2D_norm()
    {
        return mvecMatch_2D_2D_norm;
    }
    
    vector<pair<cv::Point2f, cv::Point2f>> Matcher::getMatch2D2D_pixel()
    {
        return mvecMatch_2D_2D_pixel;
    }
    
    vector<int> Matcher::getMatchedID()
    {
       return mvecMatchedID; 
    }
}



