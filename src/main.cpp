#include "LoadData.h"
#include "System.h"
#include "timer/tictoc.h"

using namespace std;


int main(int argc, char** agrv)
{
    if (argc != 5)
    {
        cerr <<"--- Error: Number of Input arguments is un-correct"<<endl;
        exit(-1);
    }
    
    const string imgPathL = agrv[1];
    const string imgPathR = agrv[2];
    const string imgTimePath = agrv[3]; 
    const string configPath = agrv[4];
    
    // step 1. load image data
    cout << "--- Loading images, please wait a while"<<endl;
    
    StereoVO::LoadData loader;
    loader.loadImgData(imgPathL, imgPathR, imgTimePath);
    // loader.test_visulization();  // test only
    
    vector<cv::Mat> vecImgL = loader.getImageDataL();
    vector<cv::Mat> vecImgR = loader.getImageDataR();
    
    if (vecImgL.size() != vecImgR.size())
    {
        cerr << "--- Error: something wrong with loaded images"<<endl;
        exit(-1);
    }
    
    // step 2. init system
    StereoVO::System VO(configPath);
    
    // step 3. image process - main loop
    cout << "--- main loop start"<<endl;
    for (int i = 0; i < vecImgL.size(); i ++)
    {
        cv::Mat imgL = vecImgL[i];
        cv::Mat imgR = vecImgR[i];
        
        VO.processStereo(imgL, imgR);
    }
    
    VO.setStopFlag();
    
    return 0;
    
}

