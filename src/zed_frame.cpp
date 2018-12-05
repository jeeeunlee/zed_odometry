#include "zed_frame.h"

bool Frame::mbInitialComputations=true;
long unsigned int Frame::nNextId=0;
double Frame::NNDRATIO = 0.75;
// float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
Ptr<FeatureDetector> Frame::mpDetector = ORB::create(); //todo : set argument
Ptr<DescriptorExtractor> Frame::mpDescriptor = ORB::create();
Ptr<DescriptorMatcher> Frame::mpMatcher = DescriptorMatcher::create("BruteForce-Hamming");

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

Frame::Frame(const Mat &imLeft, const Mat &imRight, const double &timeStamp, Frame::MatchingMethod method)
    :mTimeStamp(timeStamp)
{
    // Frame ID
    mnId=nNextId++;
    mImgLeft=imLeft.clone();
    mImgRight=imRight.clone();

    // extract features and descriptor
    mpDetector->detect ( imLeft, mKeypoints );
    mpDetector->detect ( imRight, mKeypointsRight );
    
    mpDescriptor->compute ( imLeft, mKeypoints, mDescriptors );
    mpDescriptor->compute ( imRight, mKeypointsRight, mDescriptorsRight );

    mpMatcher->match( mDescriptors, mDescriptorsRight, mMatches );
    
    // extract good match using NNDR(Nearest Neighbour Distance Ratio) 
    getGoodMatches(method); //MatchingMethod::NNDR, MatchingMethod::RANSAC
    
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
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
        {
            vector< vector<DMatch >> nnMatches;
            mpMatcher->knnMatch(mDescriptors, mDescriptorsRight, nnMatches, 2);
            for(int k = 0; k < nnMatches.size(); k++)
            {
                if(nnMatches[k][0].distance / nnMatches[k][1].distance < NNDRATIO)
                {
                    if( nnMatches[k][0].distance  <= 30.0  ) //max(2*mindist, 30.0)
                        mGoodMatches.push_back(nnMatches[k][0]);
                }
            }
            cout<< "NNDR: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << endl;
        }break;
        
        case MatchingMethod::RANSAC : // extract good match using RANSAC
        default:
        {
            vector<Point2f> alinedKeypoints, alinedKeypointsRight;
            static unsigned int numOfMatches = mMatches.size();
            for(uint k=0; k<numOfMatches; k++)
            {
                alinedKeypoints.push_back(mKeypoints[mMatches[k].queryIdx].pt);
                alinedKeypointsRight.push_back(mKeypointsRight[mMatches[k].trainIdx].pt);
            }
            vector<uchar> inliners(numOfMatches,0);

            Mat H = findFundamentalMat(alinedKeypoints, alinedKeypointsRight, inliners, FM_RANSAC, 1.1f, 0.99);

            for(uint k=0; k<inliners.size(); k++)
            {
                if (inliners[k] > 0.)
                {
                    mGoodMatches.push_back(mMatches[k]);
                }
            }
            cout<< "RANSAC: # of good Matches: " << mGoodMatches.size() << " / Matches: " << mMatches.size() << endl;

        }break;     
    }
}