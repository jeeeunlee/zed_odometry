#ifndef POINT3D_H_
#define POINT3D_H_

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include "zed_param.h"

// using namespace std;
// using namespace cv;

struct Point3D{
    float X, Y, Z;    
};

class PointReconstructor{

public:
    PointReconstructor();

    // Copy constructor
    // PointReconstructor(const PointReconstructor &point3d);

    // constructor
    PointReconstructor(ZParam *param);

    // compute for KeyPoint
    void compute(const std::vector<cv::Point2f> &kpleft, const std::vector<cv::Point2f> &kpright, std::vector<cv::Point3f> &kp3f);

    // compute undistortKeyPoint
    std::vector<cv::KeyPoint> UndistortKeyPoints(const std::vector<cv::KeyPoint> &mvKeys, const CALIBINFO &calibinfo, const cv::Mat &distCoeff);

    // get Calibration information
    void getCalibInfo();

    void printMatrix(const std::string &caption, const cv::Mat &MAT);
     

public:
    static bool mbInitialzedGetInformation;
    // static CALIBINFO LeftCalib, RightCalib;

    ZParam *mParam;
    cv::Mat mKL, mKR;
    cv::Mat mDistCoeffsL, mDistCoeffsR;
    cv::Mat mHomoLtoR, mHomoRtoL;
    
};

#endif /* POINT3D_H_ */