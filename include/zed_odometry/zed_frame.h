#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "zed_point3d.h"

// using namespace std;
// using namespace cv;

class Frame
{
public:
    enum MatchingMethod
    {
        DEFAULT=0,
        NNDR=1,
        RANSAC=2        
    };

public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, const std::string &source, MatchingMethod method);

    // get good matches
    void getGoodMatches(MatchingMethod method);
    std::vector<cv::DMatch> getGoodMatches_NNDR(double nndr_threshold=NNDR_THRESHOLD);
    std::vector<cv::DMatch> getGoodMatches_RANSAC(double error=1.1f, double confidence=0.995f);


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

    ZParam* mParam;

    PointReconstructor* mpPointReconstructor;

    // image
    cv::Mat mImgLeft, mImgRight;

    // matches between L&R
    std::vector<cv::DMatch> mMatches, mGoodMatches;

    // Vector of keypoints
    std::vector<cv::KeyPoint> mKeypoints, mKeypointsRight;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    static double NNDR_THRESHOLD;

    static cv::Ptr<cv::FeatureDetector> mpDetector;
    static cv::Ptr<cv::DescriptorExtractor> mpDescriptor;
    static cv::Ptr<cv::DescriptorMatcher> mpMatcher;


private:
    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc


};


#endif // FRAME_H