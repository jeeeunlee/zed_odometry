#include "zed_frame.h"


bool Frame::mbInitialComputations=true;
ZParam* Frame::mParam = NULL;
PointReconstructor* Frame::mpPointReconstructor = NULL;
long unsigned int Frame::nNextId=0;
double Frame::NNDR_THRESHOLD = 0.8; // the smaller, we get the fewer matches.
// float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
cv::Ptr<cv::FeatureDetector> Frame::mpDetector = cv::ORB::create(2000); //todo : set argument
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

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ZParam *param, Frame::MatchingMethod method)
    :mTimeStamp(timeStamp)
{
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        mParam = param;
        mpPointReconstructor = new PointReconstructor(param);
        mbInitialComputations=false;
    }

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

    // std::cout << "mKeypoints:" << mKeypoints.size() << std::endl;
    // std::cout << "mKeypointsRight:" << mKeypointsRight.size() << std::endl;
    // std::cout << "mDescriptors:" << mDescriptors.size() << std::endl;
    // std::cout << "mDescriptorsRight:" << mDescriptorsRight.size() << std::endl;
    
    // extract good matchs
    extractGoodMatches(method); //MatchingMethod::NNDR, MatchingMethod::RANSAC    
}

void Frame::extractGoodMatches(MatchingMethod method)
{
    switch (method)
    {
        case MatchingMethod::NNDR : // extract good match using NNDR(Nearest Neighbour Distance Ratio)            
            extractGoodMatches_NNDR(mDescriptors, mDescriptorsRight, mGoodMatches, 0.8);
            std::cout<< "NNDR: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
            break;

        case MatchingMethod::RANSAC : // extract good match using FunMatRANSAC
        {
            std::vector<cv::Point2f> alignedKps, alignedKpsRight;
            alignKeyPoint(mKeypoints, mKeypointsRight, mMatches, alignedKps, alignedKpsRight);
            extractGoodMatches_FunMatRANSAC(alignedKps, alignedKpsRight, mMatches, mGoodMatches, 1.05,0.995);
            std::cout<< "RANSAC: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
        }break;           

        case MatchingMethod::DEFAULT : 
        default: 
        {
            // extract goodMatchNNDR
            std::vector<cv::DMatch> goodMatchNNDR;          
            extractGoodMatches_NNDR(mDescriptors, mDescriptorsRight, goodMatchNNDR, 0.8);
            // align keypoint with goodMatchNNDR and extract goodMatch using FunMatRANSAC
            std::vector<cv::Point2f> alignedKps, alignedKpsRight;
            alignKeyPoint(mKeypoints, mKeypointsRight, goodMatchNNDR, alignedKps, alignedKpsRight);
            extractGoodMatches_FunMatRANSAC(alignedKps, alignedKpsRight, goodMatchNNDR, mGoodMatches, 1.05, 0.995);    
            std::cout<< "DEFAULT: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << std::endl;
        }break;        
    }
}

void Frame::extractGoodMatches_NNDR(const cv::Mat &descriptor1, const cv::Mat &descriptor2, std::vector<cv::DMatch> &output_matches, double ratio_threshold)
{
    std::vector<std::vector<cv::DMatch>> matches12, matches21;
    mpMatcher->knnMatch( descriptor1, descriptor2, matches12, 2 );
    mpMatcher->knnMatch( descriptor2, descriptor1, matches21, 2 );    

    // ratio test proposed by David Lowe paper ratio_threshold = 0.8
    std::vector<cv::DMatch> good_matches1, good_matches2;

    // Yes , the code here is redundant, it is easy to reconstruct it ....
    for(int i=0; i < matches12.size(); i++){
        if(matches12[i][0].distance < ratio_threshold * matches12[i][1].distance)
            good_matches1.push_back(matches12[i][0]);
    }

    for(int i=0; i < matches21.size(); i++){
        if(matches21[i][0].distance < ratio_threshold * matches21[i][1].distance)
            good_matches2.push_back(matches21[i][0]);
    }

    // Symmetric Test
    output_matches.clear();
    for(int i=0; i<good_matches1.size(); i++){
        for(int j=0; j<good_matches2.size(); j++){
            if(good_matches1[i].queryIdx == good_matches2[j].trainIdx && good_matches2[j].queryIdx == good_matches1[i].trainIdx){
                output_matches.push_back(cv::DMatch(good_matches1[i].queryIdx, good_matches1[i].trainIdx, good_matches1[i].distance));
                break;
            }
        }
    }

}

void Frame::extractGoodMatches_FunMatRANSAC(const std::vector<cv::Point2f> &alinedPoints, const std::vector<cv::Point2f> &alinedPoints2, const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &output_matches, double error, double confidence)
{   
    int numOfMatches = matches.size();
    std::vector<uchar> inliners(numOfMatches,0);
    cv::Mat H = findFundamentalMat(alinedPoints, alinedPoints2, inliners, cv::FM_RANSAC, error, confidence);

    output_matches.clear();
    for(uint k=0; k<inliners.size(); k++)
    {
        if (inliners[k] > 0.)        
            output_matches.push_back(matches[k]);        
    }    
}

void Frame::alignKeyPoint(const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2, const std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &out_kp1, std::vector<cv::KeyPoint> &out_kp2)
{
    unsigned int numOfMatches = matches.size();
    out_kp1.clear();
    out_kp2.clear();
    for(uint k=0; k<numOfMatches; k++)
    {
        out_kp1.push_back(kp1[matches[k].queryIdx]);
        out_kp2.push_back(kp2[matches[k].trainIdx]);
    }
}

void Frame::alignKeyPoint(const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2, const std::vector<cv::DMatch> &matches, std::vector<cv::Point2f> &out_kp1, std::vector<cv::Point2f> &out_kp2)
{
    unsigned int numOfMatches = matches.size();
    out_kp1.clear();
    out_kp2.clear();
    for(uint k=0; k<numOfMatches; k++)
    {
        out_kp1.push_back(kp1[matches[k].queryIdx].pt);
        out_kp2.push_back(kp2[matches[k].trainIdx].pt);
    }
}