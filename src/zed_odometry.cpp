#include <iostream>
#include <fstream>
#include <iomanip> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "zed_frame.h"
#include "zed_param.h"

using namespace std;
using namespace cv;
// using namespace ORB_ODOMETRY;

class Frame;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void DrawImages(const string &title, const Frame &_frame);

int main ( int argc, char** argv ) // argv[1]=calibration yaml, argv[2]=image file path
{
    // set param
    ZParam* param = ZParam::getInstance(argv[1]);

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);

    int nImages = min(vstrImageLeft.size(), vstrImageRight.size());

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }
    else
    {
        cout<<"nImages: "<< nImages <<endl;
        cout<<"ex: vstrImageLeft[0]= "<< vstrImageLeft[0]<< endl;
    }

    // Main loop
    Mat imLeft, imRight;
    Frame mCurrentFrame;
    
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        /*  CV_LOAD_IMAGE_COLOR : 이미지 파일을 Color로 읽어들입니다. 투명한 부분은 무시되며, Default값입니다.
            CV_LOAD_IMAGE_GRAYSCALE : 이미지를 Grayscale로 읽어 들입니다. 실제 이미지 처리시 중간단계로 많이 사용합니다.
            CV_LOAD_IMAGE_UNCHANGED : 이미지파일을 alpha channel까지 포함하여 읽어 들입니다.    */        
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_COLOR); 
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_COLOR);
        
        mCurrentFrame = Frame(imLeft, imRight, vTimestamps[ni], argv[1], Frame::MatchingMethod::NNDR);
        DrawImages("NNDR", mCurrentFrame);

        mCurrentFrame = Frame(imLeft, imRight, vTimestamps[ni], argv[1], Frame::MatchingMethod::RANSAC);
        DrawImages("RANSAC", mCurrentFrame);

        mCurrentFrame = Frame(imLeft, imRight, vTimestamps[ni], argv[1], Frame::MatchingMethod::DEFAULT);
        DrawImages("DEFAULT", mCurrentFrame);

        waitKey(0);


    }   
    
    return 0;
}

void DrawImages(const string &title, const Frame &_frame){

    // Mat img;
    // drawMatches ( _frame.mImgLeft, _frame.mKeypoints, _frame.mImgRight, _frame.mKeypointsRight, _frame.mMatches, img );
    // imshow ( "matches", img );

    Mat img_goodmatch;
    drawMatches ( _frame.mImgLeft, _frame.mKeypoints, _frame.mImgRight, _frame.mKeypointsRight, _frame.mGoodMatches, img_goodmatch );
    namedWindow(title.c_str(), WINDOW_NORMAL);
    resizeWindow( title.c_str(), 1200, 400);
    imshow ( title.c_str(), img_goodmatch );
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/left";
    string strPrefixRight = strPathToSequence + "/right";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i+1;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
