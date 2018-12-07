#include "zed_frame.h"


bool Frame::mbInitialComputations=true;
long unsigned int Frame::nNextId=0;
double Frame::NNDR_THRESHOLD = 0.75;
// float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
cv::Ptr<cv::FeatureDetector> Frame::mpDetector = cv::ORB::create(); //todo : set argument
cv::Ptr<cv::DescriptorExtractor> Frame::mpDescriptor = cv::ORB::create();
cv::Ptr<cv::DescriptorMatcher> Frame::mpMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mTimeStamp(frame.mTimeStamp), mImgLeft(frame.mImgLeft.clone()), mImgRight(frame.mImgRight.clone()),
    // mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
    mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone())
{
    for(int i=0;i<frame.mMatches.size();i++)
        mMatches[i]=frame.mMatches[i];

    for(int i=0;i<frame.mKeypoints.size();i++)
        mKeypoints[i]=frame.mKeypoints[i];
    
    for(int i=0;i<frame.mKeypointsRight.size();i++)
        mKeypointsRight[i]=frame.mKeypointsRight[i];
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, const std::string &source, Frame::MatchingMethod method)
    :mTimeStamp(timeStamp)
{
    // Frame ID
    mnId=nNextId++;
    //std::cout<< "mnId=" << mnId<< ", nNextId=" << nNextId<< std::endl;

    mImgLeft=imLeft.clone();
    mImgRight=imRight.clone();

    // extract features and descriptor
    mpDetector->detect( imLeft, mKeypoints );
    mpDetector->detect( imRight, mKeypointsRight );
    
    mpDescriptor->compute( imLeft, mKeypoints, mDescriptors );
    mpDescriptor->compute( imRight, mKeypointsRight, mDescriptorsRight );

    mpMatcher->match( mDescriptors, mDescriptorsRight, mMatches );
    
    // extract good match using NNDR(Nearest Neighbour Distance Ratio) 
    getGoodMatches(method); //MatchingMethod::NNDR, MatchingMethod::RANSAC
    
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        std::cout<<"im here!!!"<<std::endl;
        mParam = ZParam::getInstance(source);
        std::cout<<"im here2222"<<std::endl;
        mpPointReconstructor = new PointReconstructor(source);
        // ComputeImageBounds(imLeft);

        // mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        // mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        // fx = K.at<float>(0,0);
        // fy = K.at<float>(1,1);
        // cx = K.at<float>(0,2);
        // cy = K.at<float>(1,2);
        // invfx = 1.0f/fx;
        // invfy = 1.0f/fy;
        mbInitialComputations=false;
    }

}

void Frame::getGoodMatches(MatchingMethod method)
{
    switch (method)
    {
        case MatchingMethod::NNDR : // extract good match using NNDR(Nearest Neighbour Distance Ratio)            
            mGoodMatches=getGoodMatches_NNDR();
            std::cout<< "NNDR: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
            break;

        case MatchingMethod::RANSAC : // extract good match using RANSAC
            mGoodMatches=getGoodMatches_RANSAC(1.05,0.995);
            std::cout<< "RANSAC: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
            break;     
        case MatchingMethod::DEFAULT : 
        default:
            std::vector<cv::DMatch> matchTemp=mMatches;
            mMatches.clear(); mGoodMatches.clear();          
            mMatches=getGoodMatches_NNDR(0.8);            
            mGoodMatches=getGoodMatches_RANSAC(1.05,0.995);
            mMatches.clear(); mMatches=matchTemp;
            std::cout<< "DEFAULT: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
            break;
    }
}

std::vector<cv::DMatch> Frame::getGoodMatches_NNDR(double nndr_threshold)
{
    std::vector<cv::DMatch> goodMatches;
    std::vector<std::vector<cv::DMatch>> nnMatches;
    mpMatcher->knnMatch(mDescriptors, mDescriptorsRight, nnMatches, 2);
    for(int k = 0; k < nnMatches.size(); k++)
    {
        if(nnMatches[k][0].distance  < nndr_threshold *  nnMatches[k][1].distance)
        {
            if( nnMatches[k][0].distance  <= 20.0  ) //max(2*mindist, 30.0)
                goodMatches.push_back(nnMatches[k][0]);
        }
    }    
    return goodMatches;
}

std::vector<cv::DMatch> Frame::getGoodMatches_RANSAC(double error, double confidence)
{
    std::vector<cv::DMatch> goodMatches;
    std::vector<cv::Point2f> alinedKeypoints, alinedKeypointsRight;
    unsigned int numOfMatches = mMatches.size();
    for(uint k=0; k<numOfMatches; k++)
    {
        alinedKeypoints.push_back(mKeypoints[mMatches[k].queryIdx].pt);
        alinedKeypointsRight.push_back(mKeypointsRight[mMatches[k].trainIdx].pt);
    }

    Point3D p1=mpPointReconstructor->compute(alinedKeypoints[0], alinedKeypointsRight[0]);

    std::vector<uchar> inliners(numOfMatches,0);
    cv::Mat H = findFundamentalMat(alinedKeypoints, alinedKeypointsRight, inliners, cv::FM_RANSAC, error, confidence);

    for(uint k=0; k<inliners.size(); k++)
    {
        if (inliners[k] > 0.)
        {
            goodMatches.push_back(mMatches[k]);
        }
    }    
    return goodMatches;
}

