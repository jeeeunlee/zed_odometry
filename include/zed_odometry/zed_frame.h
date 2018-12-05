#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Frame
{
public:
    enum MatchingMethod
    {
        NNDR=1,
        RANSAC=2
    };

public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, MatchingMethod method);

    // get good matches
    void getGoodMatches(MatchingMethod method);


public:
    static bool mbInitialComputations;

    // Frame timestamp.
    double mTimeStamp;

    /*
    // Calibration matrix and OpenCV distortion parameters.
    Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;
    */

    // image
    Mat mImgLeft, mImgRight;

    // matches between L&R
    vector<DMatch> mMatches, mGoodMatches;

    // Vector of keypoints
    vector<KeyPoint> mKeypoints, mKeypointsRight;

    // ORB descriptor, each row associated to a keypoint.
    Mat mDescriptors, mDescriptorsRight;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    static double NNDRATIO;

    static Ptr<FeatureDetector> mpDetector;
    static Ptr<DescriptorExtractor> mpDescriptor;
    static Ptr<DescriptorMatcher> mpMatcher;


private:
    // Rotation, translation and camera center
    Mat mRcw;
    Mat mtcw;
    Mat mRwc;
    Mat mOw; //==mtwc


};


#endif // FRAME_H