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

Frame::Frame(const IMAGE &imLeft, const IMAGE &imRight, const double &timeStamp, ZParam *param, Frame::MatchingMethod method)
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
    extractGoodMatches(method, mDescriptors, mDescriptorsRight, mKeypoints, mKeypointsRight, mMatches, mGoodMatches);
}

void Frame::extractGoodMatches(MatchingMethod method, const DESCRIPTORS &dcr1, const DESCRIPTORS &dcr2, const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &inputMatches, V_MATCHES &outputGoodMatches)
{
    switch (method)
    {
        case MatchingMethod::NNDR : // extract good match using NNDR(Nearest Neighbour Distance Ratio)            
            extractGoodMatches_NNDR(dcr1, dcr2, outputGoodMatches, 0.8);
            //std::cout<< "NNDR: # of good Matches: " << outputGoodMatches.size() << " / Matches: " << inputMatches.size() << std::endl;
            break;

        case MatchingMethod::FunMatRANSAC : // extract good match using FunMatRANSAC
        {            
            extractGoodMatches_FunMatRANSAC(kp1, kp2, inputMatches, outputGoodMatches, 1.02,0.995);
            //std::cout<< "FunMat: # of good Matches: " << outputGoodMatches.size() << " / Matches: " << inputMatches.size() << std::endl;
        }break;
        case MatchingMethod::HomoRANSAC :
        {
            extractGoodMatches_HomographyRANSAC(kp1, kp2, inputMatches, outputGoodMatches, 2000, 1.02,0.995);
            //std::cout<< "Homography: # of good Matches: " << outputGoodMatches.size() << " / Matches: " << inputMatches.size() << std::endl;

        }break;
        case MatchingMethod::DEFAULT : 
        default: 
        {
            // extract goodMatchNNDR
            V_MATCHES goodMatchesNNDR;          
            extractGoodMatches_NNDR(dcr1, dcr2, goodMatchesNNDR, 0.8);

            // align keypoint with goodMatchesNNDR and extract goodMatch using FunMatRANSAC
            extractGoodMatches_FunMatRANSAC(kp1, kp2, goodMatchesNNDR, outputGoodMatches, 1.02, 0.995);    
            //std::cout<< "DEFAULT: # of good Matches: " << outputGoodMatches.size() << " / Matches: " << inputMatches.size() << std::endl;
        }break;        
    }
}

void Frame::extractGoodMatches_NNDR(const DESCRIPTORS &descriptor1, const DESCRIPTORS &descriptor2, V_MATCHES &output_matches, double ratio_threshold)
{
    std::vector<V_MATCHES> matches12, matches21;
    mpMatcher->knnMatch( descriptor1, descriptor2, matches12, 2 );
    mpMatcher->knnMatch( descriptor2, descriptor1, matches21, 2 );    

    // ratio test proposed by David Lowe paper ratio_threshold = 0.8
    V_MATCHES good_matches1, good_matches2;
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

void Frame::extractGoodMatches_FunMatRANSAC(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_MATCHES &output_matches, double error, double confidence)
{   
    V_POINT2F alinedPoints, alinedPoints2;
    alignKeyPoint(kp1, kp2, matches, alinedPoints, alinedPoints2);

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

void Frame::extractGoodMatches_HomographyRANSAC(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_MATCHES &output_matches, const int maxIters, const double error, const double confidence)
{   
    V_POINT2F alinedPoints, alinedPoints2;
    alignKeyPoint(kp1, kp2, matches, alinedPoints, alinedPoints2);

    int numOfMatches = matches.size();
    std::vector<uchar> inliners(numOfMatches,0);
    // method; cv::RHO(PROSAC), cv::RANSAC
    cv::Mat H = findHomography(alinedPoints, alinedPoints2, cv::RANSAC, error, inliners, maxIters, confidence);

    output_matches.clear();
    for(uint k=0; k<inliners.size(); k++)
    {
        if (inliners[k] > 0.)        
            output_matches.push_back(matches[k]);        
    }    
}

void Frame::alignKeyPoint(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_KEYPOINTS &out_kp1, V_KEYPOINTS &out_kp2)
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

void Frame::alignKeyPoint(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_POINT2F &out_kp1, V_POINT2F &out_kp2)
{
    unsigned int numOfMatches = matches.size();
    out_kp1.clear();
    out_kp2.clear();
    for(uint k=0; k<numOfMatches; k++)
    {
        out_kp1.push_back(kp1[matches[k].queryIdx].pt);
        out_kp2.push_back(kp2[matches[k].trainIdx].pt);
    }

    std::string filepath = "/home/lee/catkin_ws/src/zed_odometry/out/";
    std::ostringstream ost1, ost2;
    ost1 << filepath << "feature2d_l_" << mnId << ".txt";
    ost2 << filepath << "feature2d_r_" << mnId << ".txt";

    std::cout<< ost1.str() << std::endl;

    mpPointReconstructor->save(out_kp1, ost1.str().c_str());
    mpPointReconstructor->save(out_kp2, ost2.str().c_str());
}

void Frame::alignDescriptor(const DESCRIPTORS &dsctr1, const DESCRIPTORS &dsctr2, const V_MATCHES &matches, DESCRIPTORS &out_dsctr1, DESCRIPTORS &out_dsctr2)
{
    unsigned int numOfMatches = matches.size();
    unsigned int descriptor_size = dsctr1.rows;
    out_dsctr1=cv::Mat(descriptor_size, numOfMatches, CV_32F);
    out_dsctr2=cv::Mat(descriptor_size, numOfMatches, CV_32F);
    for(uint k=0; k<numOfMatches; k++)
    {
        // out_dsctr1.push_back(dsctr1[matches[k].queryIdx].pt);
        // out_dsctr2.push_back(dsctr2[matches[k].trainIdx].pt);
    }
}

void Frame::reconstruct3D()
{
    V_POINT2F alinedPoints, alinedPoints2;
    alignKeyPoint(mKeypoints, mKeypointsRight, mGoodMatches, alinedPoints, alinedPoints2);
    mKeypoints3d.clear();
    mpPointReconstructor->compute(alinedPoints, alinedPoints2, mKeypoints3d);

    // process : eliminate points that have large reprojection error  
    V_POINT2F reproPoints, reproPoints2;
    mpPointReconstructor->reprojection(mKeypoints3d, reproPoints, reproPoints2);

    uint numOfPoints = reproPoints.size();
    float dL, dR;
    for(uint i=0; i<numOfPoints; i++){
        dL = cv::norm(reproPoints[i] - alinedPoints[i]);
        dR = cv::norm(reproPoints2[i] - alinedPoints2[i]);
        // std::cout<<dL << ","<<dR << "/";
    }
    // std::cout<<std::endl;;
    
}

void Frame::getOdometryUsingPnP(const Frame &_frame_prev)
{  
    // extract goodMatches btw t-1 and t left image
    V_MATCHES matchesLeft;
    mpMatcher->match( _frame_prev.mDescriptors, mDescriptors, matchesLeft );
    V_MATCHES goodMatchesPrevLeft;
    extractGoodMatches(DEFAULT, _frame_prev.mDescriptors, mDescriptors, _frame_prev.mKeypoints, mKeypoints, matchesLeft, goodMatchesPrevLeft);
    
    // matching pairs
    // same keypoint in t-1 left image if _frame_prev.mGoodMatches.queryIdx == goodMatchesPrevLeft.queryIdx
    V_POINT3F objectPoints;	// 3d world coordinates
    V_POINT2F imagePoints;	// 2d image coordinates

    uint numOfMatches = goodMatchesPrevLeft.size();
    uint numOfgoodMatches = _frame_prev.mGoodMatches.size();
    for(uint k=0; k<numOfMatches; k++)
    {
        for(uint j=0; j<numOfgoodMatches; j++)
        {
            if(_frame_prev.mGoodMatches[j].queryIdx == goodMatchesPrevLeft[k].queryIdx)
            {
                objectPoints.push_back(_frame_prev.mKeypoints3d[j]);
                imagePoints.push_back(mKeypoints[goodMatchesPrevLeft[k].trainIdx].pt);
                mGoodMatchesPrevLeft.push_back(goodMatchesPrevLeft[k]);
            }
        }
    }
    //V_MATCHES MatchesRight;
    //_frame_prev.mpMatcher->match( _frame_prev.mDescriptorsRight, mDescriptorsRight, MatchesRight );
    // std::cout << "solvePnP: matches btw t-1,t : " << numOfMatches << " / t-1 3dPoint : " << numOfgoodMatches << " / final :" << imagePoints.size() << std::endl;
    
    mpPointReconstructor->solvePnP(objectPoints, imagePoints, true);    
}