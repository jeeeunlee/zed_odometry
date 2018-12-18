#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "zed_point3d.h"
#include "zed_common.h"

// using namespace std;
// using namespace cv;

class Frame
{
public:
    enum MatchingMethod
    {
        DEFAULT=0,
        NNDR=1,
        FunMatRANSAC=2,
        HomoRANSAC=3        
    };

public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const IMAGE &imLeft, const IMAGE &imRight, const double &timeStamp, ZParam *param, MatchingMethod method);

    // extract good matches
    void extractGoodMatches(MatchingMethod method);
    void extractGoodMatches(MatchingMethod method, const DESCRIPTORS &dcr1, const DESCRIPTORS &dcr2, const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &inputMatches, V_MATCHES &outputGoodMatches);

    void extractGoodMatches_NNDR(const cv::Mat &descriptor1, const cv::Mat &descriptor2, V_MATCHES &output_matches, double ratio_threshold=NNDR_THRESHOLD);
    void extractGoodMatches_FunMatRANSAC(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_MATCHES &output_matches, double error=1.1f, double confidence=0.995f);
    void extractGoodMatches_HomographyRANSAC(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_MATCHES &output_matches, const int maxIters=2000, const double error=1.1f, const double confidence=0.995f);
    
    // get aligned keyPoints
    void alignKeyPoint(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_POINT2F &out_kp1, V_POINT2F &out_kp2);
    void alignKeyPoint(const V_KEYPOINTS &kp1, const V_KEYPOINTS &kp2, const V_MATCHES &matches, V_KEYPOINTS &out_kp1, V_KEYPOINTS &out_kp2);

    // get alined descriptors
    void alignDescriptor(const DESCRIPTORS &dsctr1, const DESCRIPTORS &dsctr2, const V_MATCHES &matches, DESCRIPTORS &out_dsctr1, DESCRIPTORS &out_dsctr2);


    // get 3d points
    void reconstruct3D();

    //
    void solvePnP();

public:
    static bool mbInitialComputations;

    // Frame timestamp.
    double mTimeStamp;

    // calibration parameters and functions
    static ZParam* mParam;
    static PointReconstructor* mpPointReconstructor;

    // image
    IMAGE mImgLeft, mImgRight;

    // matches between L&R
    V_MATCHES mMatches, mGoodMatches;

    // Vector of keypoints
    V_KEYPOINTS mKeypoints, mKeypointsRight;

    // Vector of aliend keypoints
    V_POINT2F mAlinedPoints, mAlinedPointsRights;

    // ORB descriptor, each row associated to a keypoint. size:[32 x N]
    DESCRIPTORS mDescriptors, mDescriptorsRight;

    // 3d keypoints
    V_POINT3F mKeypoints3d;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    static double NNDR_THRESHOLD;

    static cv::Ptr<cv::FeatureDetector> mpDetector;
    static cv::Ptr<cv::DescriptorExtractor> mpDescriptor;
    static cv::Ptr<cv::DescriptorMatcher> mpMatcher;


private:
    // Rotation, translation and camera center
    cv::Mat mRcw; //camera to world ??
    cv::Mat mtcw;
    cv::Mat mRwc; //world to camera?
    cv::Mat mOw; //==mtwc
};


#endif // FRAME_H